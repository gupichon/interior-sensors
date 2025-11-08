# print("Hello World!")
import time, json, board, busio, wifi, socketpool, alarm
import adafruit_minimqtt.adafruit_minimqtt as MQTT
import adafruit_tmp117

try:
    import adafruit_scd4x
except ImportError:
    adafruit_scd4x = None

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

# ---------- I2C & capteurs ----------
#i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
i2c = board.STEMMA_I2C()
tmp = adafruit_tmp117.TMP117(i2c)  # 0x48

scd = None
if adafruit_scd4x is not None:
    try:
        scd = adafruit_scd4x.SCD4X(i2c)  # 0x62
        print("Serial number:", [hex(i) for i in scd.serial_number])
        try:
            scd.start_periodic_measurement()  # reste actif entre réveils
        except Exception as ex:
            print(ex)
    except Exception as ex2:
        scd = None
        print(ex2)

def read_scd_low_power(max_wait_s=4.0):
    print("Reading scd...")
    if scd is None:
        return {}
    t0 = time.monotonic()
    while not scd.data_ready and (time.monotonic() - t0) < max_wait_s:
        time.sleep(0.2)
    out = {}
    if scd.data_ready:
        print("Data ready !!")
        if scd.CO2 is not None:
            out["co2"] = int(scd.CO2)
        if scd.relative_humidity is not None:
            out["humidity"] = float(scd.relative_humidity)
        if scd.temperature is not None:
            out["temperature_sdc"] = float(scd.temperature)
    else:
        print("No data :(")

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
    """Publie la découverte Home Assistant une seule fois (flag en sleep_memory)."""
    # Flag: si alarm.sleep_memory[0] != 0xA5, publier; sinon sauter.
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
        "model": "QT Py ESP32-S3 + TMP117 + SCD41"
    }

    def pub_config(suffix, name, unit, dev_cla, value_tpl):
        cfg = {
            "name": f"{DEVICE_NAME} {name}",
            "uniq_id": f"{device_id}_{suffix}",
            "stat_t": topic_state,
            "unit_of_meas": unit,
            "dev_cla": dev_cla,
            "val_tpl": value_tpl,
            "dev": device_info,
            "avty_t": topic_avail,
        }
        cli.publish(f"{HA_PREFIX}/sensor/{device_id}_{suffix}/config", json.dumps(cfg), retain=True)

    # Température
    pub_config("temp", "Température", "°C", "temperature", "{{ value_json.temperature }}")
    # Humidité
    pub_config("hum", "Humidité", "%", "humidity", "{{ value_json.humidity }}")
    # CO2
    pub_config("co2", "CO₂", "ppm", "carbon_dioxide", "{{ value_json.co2 }}")
    # Température via le SDC-41
    pub_config("temp_sdc", "Température SDC-41", "°C", "temperature_sdc", "{{ value_json.temp_sdc }}")

    # marque comme publié
    try:
        alarm.sleep_memory[0] = 0xA5
    except Exception:
        pass

# ---------- Main ----------
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

    try:
        payload["temperature"] = float(tmp.temperature)
    except Exception as e:
        print("TMP117 err:", e)

    payload.update(read_scd_low_power(max_wait_s=10))

    if any(k in payload for k in ("temperature", "humidity", "co2", "temperature_sdc")):
        mqtt.publish(TOPIC_STATE, json.dumps(payload), retain=False)
        print("MQTT ->", payload)

    mqtt.disconnect()

except Exception as e:
    print("Erreur:", e)
    try:
        if mqtt:
            mqtt.disconnect()
    except Exception:
        pass

# ---------- Deep sleep ----------
print(f"Deep sleep {SLEEP_S} s …")
#time_alarm = alarm.time.TimeAlarm(monotonic_time=time.monotonic() + SLEEP_S)
#alarm.exit_and_deep_sleep_until_alarms(time_alarm)

