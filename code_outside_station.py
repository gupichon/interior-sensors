import time, json, board, wifi, socketpool, alarm
import adafruit_minimqtt.adafruit_minimqtt as MQTT
import adafruit_tmp117
import neopixel

# --- Imports optionnels par capteur ---
try:
    import adafruit_scd4x
except ImportError:
    adafruit_scd4x = None

try:
    from adafruit_pm25.i2c import PM25_I2C
except ImportError:
    PM25_I2C = None

try:
    import adafruit_bme680
except ImportError:
    adafruit_bme680 = None

try:
    import adafruit_sgp30
except ImportError:
    adafruit_sgp30 = None

from secrets import secrets

SLEEP_S = int(secrets.get("publish_period", 60))
HA_PREFIX = secrets.get("ha_discovery_prefix", "homeassistant")
DEVICE_NAME = secrets.get("device_name", "Capteur IAQ")
DEVICE_ID_BASE = secrets.get("device_id_base", "iaq_device")
TOPIC_BASE = secrets.get("mqtt_topic_base", "home/iaq")

# ---------- Utilitaires ----------
def mac_suffix():
    m = wifi.radio.mac_address  # bytes(6)
    return "".join(f"{b:02x}" for b in m[-3:])  # 6 hex chars

def build_device_id():
    return f"{DEVICE_ID_BASE}_{mac_suffix()}"

def topics_for(device_id):
    return (
        f"{TOPIC_BASE}/{device_id}/state",
        f"{TOPIC_BASE}/{device_id}/availability",
    )

def r(v, nd=1):
    try:
        return round(float(v), nd)
    except Exception:
        return None

# ---------- I2C & capteurs ----------
i2c = board.STEMMA_I2C()

# TMP117 (T haute précision)
tmp = adafruit_tmp117.TMP117(i2c)  # 0x48

# SCD41 (CO2, T, RH)
scd = None
if adafruit_scd4x is not None:
    try:
        scd = adafruit_scd4x.SCD4X(i2c)  # 0x62
        print("SCD4x Serial:", [hex(i) for i in scd.serial_number])
        try:
            scd.start_periodic_measurement()  # mesure continue entre réveils
        except Exception as ex:
            print("SCD4x start:", ex)
    except Exception as ex2:
        scd = None
        print("SCD4x init:", ex2)

# PMSA003I (PM)
pm25 = None
if PM25_I2C is not None:
    try:
        pm25 = PM25_I2C(i2c, reset_pin=None)  # 0x12
    except Exception as ex:
        pm25 = None
        print("PM25 init:", ex)

# BME688 (T, RH, P, Gas)
bme = None
if adafruit_bme680 is not None:
    try:
        bme = adafruit_bme680.Adafruit_BME680_I2C(i2c)  # 0x77 (ou 0x76)
        bme.sea_level_pressure = 1013.25
    except Exception as ex:
        bme = None
        print("BME688 init:", ex)

# SGP30 (eCO2, TVOC)
sgp = None
if adafruit_sgp30 is not None:
    try:
        sgp = adafruit_sgp30.Adafruit_SGP30(i2c)
        sgp.iaq_init()  # obligatoire avant mesures
    except Exception as ex:
        sgp = None
        print("SGP30 init:", ex)

# ---------- Aides mesures ----------
def read_scd_low_power(max_wait_s=6.0):
    if scd is None:
        return {}
    t0 = time.monotonic()
    while not scd.data_ready and (time.monotonic() - t0) < max_wait_s:
        time.sleep(0.2)
    out = {}
    if scd.data_ready:
        try:
            if scd.CO2 is not None:
                out["co2"] = int(scd.CO2)
        except Exception:
            pass
        try:
            if scd.relative_humidity is not None:
                out["humidity_scd"] = r(scd.relative_humidity, 1)
        except Exception:
            pass
        try:
            if scd.temperature is not None:
                out["temperature_scd"] = r(scd.temperature, 1)
        except Exception:
            pass
    else:
        print("SCD4x: no data")
    return out

def read_pm25():
    if pm25 is None:
        return {}
    try:
        data = pm25.read()
    except Exception as ex:
        print("PM25 read:", ex)
        return {}
    # µg/m3 (prefer env if present)
    pm1 = data.get("pm10 env") or data.get("pm10 standard")
    pm25v = data.get("pm25 env") or data.get("pm25 standard")
    pm10 = data.get("pm100 env") or data.get("pm100 standard")
    out = {}
    if pm1 is not None:   out["pm1_0"] = int(pm1)
    if pm25v is not None: out["pm2_5"] = int(pm25v)
    if pm10 is not None:  out["pm10"]  = int(pm10)
    # Comptages (part/0.1L)
    for k_from, k_to in [
        ("particles 03um", "n0_3"),
        ("particles 05um", "n0_5"),
        ("particles 10um", "n1_0"),
        ("particles 25um", "n2_5"),
        ("particles 50um", "n5_0"),
        ("particles 100um", "n10"),
    ]:
        v = data.get(k_from)
        if v is not None:
            out[k_to] = int(v)
    return out

def read_bme():
    if bme is None:
        return {}
    out = {}
    try:
        out["temperature_bme"] = r(bme.temperature, 1)   # °C
        out["humidity_bme"]    = r(bme.humidity, 1)      # %RH
        out["pressure_bme"]    = r(bme.pressure, 1)      # hPa
        try:
            out["gas_resistance_bme"] = int(bme.gas)     # Ω
        except Exception:
            pass
    except Exception as ex:
        print("BME read:", ex)
    return out

def _set_sgp30_humidity_compensation():
    """Configure la compensation humidité SGP30 en priorité avec TMP117 pour la température."""
    if sgp is None:
        return

    temp = None
    rh = None

    # 1. Température depuis TMP117 si dispo
    try:
        temp = float(tmp.temperature)
    except Exception:
        temp = None

    # 2. Humidité depuis SCD41 en priorité, sinon BME688
    if scd is not None and getattr(scd, "relative_humidity", None) is not None:
        rh = float(scd.relative_humidity)
    elif bme is not None and getattr(bme, "humidity", None) is not None:
        rh = float(bme.humidity)

    # 3. Si TMP117 absent, on peut prendre la température d’un autre capteur
    if temp is None:
        if scd is not None and getattr(scd, "temperature", None) is not None:
            temp = float(scd.temperature)
        elif bme is not None and getattr(bme, "temperature", None) is not None:
            temp = float(bme.temperature)

    if temp is None or rh is None:
        return

    try:
        if hasattr(sgp, "set_iaq_relative_humidity"):
            sgp.set_iaq_relative_humidity(temp, rh)
        else:
            # calcul humidité absolue pour API legacy
            import math
            es = 6.112 * math.exp((17.62 * temp) / (243.12 + temp))  # hPa
            ah = (216.7 * (rh / 100.0) * es) / (273.15 + temp)       # g/m3
            sgp.set_iaq_humidity(ah)
    except Exception as ex:
        print("SGP30 RH comp:", ex)

def _sgp30_baseline_store_or_restore():
    """Stocke/restore baseline via alarm.sleep_memory.
       Mémoire: [1]=flag, [2..3]=eCO2 MSB..LSB, [4..5]=TVOC MSB..LSB
    """
    if sgp is None:
        return
    try:
        # restore si flag = 0xB1
        if alarm.sleep_memory[1] == 0xB1:
            be = (alarm.sleep_memory[2] << 8) | alarm.sleep_memory[3]
            bt = (alarm.sleep_memory[4] << 8) | alarm.sleep_memory[5]
            if be and bt:
                sgp.set_iaq_baseline(be, bt)
                print("SGP30 baseline restored:", be, bt)
    except Exception:
        pass

def _sgp30_baseline_save_if_valid():
    if sgp is None:
        return
    try:
        be = sgp.baseline_eCO2
        bt = sgp.baseline_TVOC
        if be and bt:
            alarm.sleep_memory[1] = 0xB1
            alarm.sleep_memory[2] = (be >> 8) & 0xFF
            alarm.sleep_memory[3] = be & 0xFF
            alarm.sleep_memory[4] = (bt >> 8) & 0xFF
            alarm.sleep_memory[5] = bt & 0xFF
            print("SGP30 baseline saved:", be, bt)
    except Exception as ex:
        print("SGP30 baseline save:", ex)

def read_sgp30():
    if sgp is None:
        return {}
    out = {}
    try:
        # compensation humidité avant mesure
        _set_sgp30_humidity_compensation()
        eco2, tvoc = sgp.iaq_measure()
        if eco2 is not None:
            out["eco2_sgp"] = int(eco2)  # ppm
        if tvoc is not None:
            out["tvoc"] = int(tvoc)      # ppb
    except Exception as ex:
        print("SGP30 read:", ex)
    return out

# ---------- Wi‑Fi & MQTT ----------
def mqtt_client():
    wifi.radio.connect(secrets["ssid"], secrets["password"])
    pool = socketpool.SocketPool(wifi.radio)
    cli = MQTT.MQTT(
        broker=secrets["mqtt_broker"],
        port=secrets.get("mqtt_port", 1883),
        username=secrets.get("mqtt_user"),
        password=secrets.get("mqtt_password"),
        socket_pool=pool,
        ssl_context=None
    )
    cli.connect()
    return cli

def publish_ha_discovery(cli, device_id, topic_state, topic_avail):
    """Publie la découverte Home Assistant une seule fois (flag en sleep_memory[0])."""
    try:
        published = (alarm.sleep_memory[0] == 0xA5)
    except Exception:
        published = False
    if published:
        return

    device_info = {
        "identifiers": [device_id],
        "name": DEVICE_NAME,
        "manufacturer": "DIY",
        "model": "QT Py ESP32-S3 + TMP117 + SCD41 + PMSA003I + BME688 + SGP30",
    }

    def pub_config(suffix, name, unit, dev_cla, value_tpl, state_class="measurement"):
        cfg = {
            "name": f"{DEVICE_NAME} {name}",
            "uniq_id": f"{device_id}_{suffix}",
            "stat_t": topic_state,
            "unit_of_meas": unit,
            "dev_cla": dev_cla,
            "val_tpl": value_tpl,
            "dev": device_info,
            "avty_t": topic_avail,
            "stat_cla": state_class,
        }
        cli.publish(f"{HA_PREFIX}/sensor/{device_id}_{suffix}/config", json.dumps(cfg), retain=True)

    # Température principale (TMP117)
    pub_config("temp", "Température", "°C", "temperature", "{{ value_json.temperature }}")

    # SCD41
    pub_config("hum_scd", "Humidité (SCD41)", "%", "humidity", "{{ value_json.humidity_scd }}")
    pub_config("co2", "CO₂ (SCD41)", "ppm", "carbon_dioxide", "{{ value_json.co2 }}")
    pub_config("temp_scd", "Température (SCD41)", "°C", "temperature", "{{ value_json.temperature_scd }}")

    # BME688
    pub_config("temp_bme", "Température (BME688)", "°C", "temperature", "{{ value_json.temperature_bme }}")
    pub_config("hum_bme", "Humidité (BME688)", "%", "humidity", "{{ value_json.humidity_bme }}")
    pub_config("pres_bme", "Pression (BME688)", "hPa", "pressure", "{{ value_json.pressure_bme }}")
    pub_config("gas_bme", "Résistance gaz (BME688)", "Ω", None, "{{ value_json.gas_resistance_bme }}")

    # PMSA003I (PM)
    pub_config("pm1_0", "PM1.0", "µg/m³", None, "{{ value_json.pm1_0 }}")
    pub_config("pm2_5", "PM2.5", "µg/m³", "pm25", "{{ value_json.pm2_5 }}")
    pub_config("pm10", "PM10", "µg/m³", "pm10", "{{ value_json.pm10 }}")
    for key, label in [
        ("n0_3", "Counts ≥0.3µm"),
        ("n0_5", "Counts ≥0.5µm"),
        ("n1_0", "Counts ≥1.0µm"),
        ("n2_5", "Counts ≥2.5µm"),
        ("n5_0", "Counts ≥5.0µm"),
        ("n10",  "Counts ≥10µm"),
    ]:
        pub_config(key, f"Particles {label}", "count/0.1L", None, f"{{{{ value_json.{key} }}}}")

    # SGP30
    pub_config("eco2_sgp", "eCO₂ (SGP30)", "ppm", "carbon_dioxide", "{{ value_json.eco2_sgp }}")
    pub_config("tvoc", "TVOC (SGP30)", "ppb", None, "{{ value_json.tvoc }}")

    # marque comme publié
    try:
        alarm.sleep_memory[0] = 0xA5
    except Exception:
        pass

# ---------- Main ----------
pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
device_id = build_device_id()
TOPIC_STATE, TOPIC_AVAIL = topics_for(device_id)

mqtt = None
try:
    mqtt = mqtt_client()
    mqtt.publish(TOPIC_AVAIL, "online", retain=True)

    publish_ha_discovery(mqtt, device_id, TOPIC_STATE, TOPIC_AVAIL)

    payload = {
        "device": device_id,
        "name": DEVICE_NAME,
        "timestamp": time.time(),
    }

    # TMP117
    try:
        t = r(tmp.temperature, 1)
        if t is not None:
            payload["temperature"] = t
    except Exception as e:
        print("TMP117 err:", e)

    # SCD41
    payload.update(read_scd_low_power(max_wait_s=10))

    # BME688
    payload.update(read_bme())

    # PM
    payload.update(read_pm25())

    # SGP30: restore baseline si dispo, mesure, puis sauvegarde
    _sgp30_baseline_store_or_restore()
    payload.update(read_sgp30())
    _sgp30_baseline_save_if_valid()

    if any(k in payload for k in (
        "temperature",
        "humidity_scd", "co2", "temperature_scd",
        "temperature_bme", "humidity_bme", "pressure_bme", "gas_resistance_bme",
        "pm1_0", "pm2_5", "pm10", "n0_3", "n0_5", "n1_0", "n2_5", "n5_0", "n10",
        "eco2_sgp", "tvoc",
    )):
        mqtt.publish(TOPIC_STATE, json.dumps(payload), retain=False)
        print("MQTT ->", payload)

    mqtt.disconnect()
    pixel.fill((0, 0, 255))
    time.sleep(0.1)
    pixel.fill((0, 0, 0))
    time.sleep(0.1)
    pixel.fill((0, 0, 255))
    time.sleep(0.1)
    pixel.fill((0, 0, 0))

except Exception as e:
    print("Erreur:", e)
    try:
        if mqtt:
            mqtt.disconnect()
    except Exception:
        pass

pixel.fill((0, 0, 0))

# ---------- Deep sleep ----------
print(f"Deep sleep {SLEEP_S} s …")
time_alarm = alarm.time.TimeAlarm(monotonic_time=time.monotonic() + SLEEP_S)
alarm.exit_and_deep_sleep_until_alarms(time_alarm)
