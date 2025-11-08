secrets = {
    "ssid": "Livebox-5BC0",
    "password": "cqWU6ijLKUH67C5KPH",

    "mqtt_broker": "192.168.1.19",
    "mqtt_port": 1883,
    "mqtt_user": "mqtt_user",
    "mqtt_password": "X)z!j5o6jq@JZtSO",

    # Topics de base: les topics complets seront
    # home/iaq/<device_id>/state  et  home/iaq/<device_id>/availability
    "mqtt_topic_base": "home/iaq",

    # MQTT Discovery
    "ha_discovery_prefix": "homeassistant",

    # Nom lisible dans HA (à personnaliser par carte)
    # Exemples: "Capteur IAQ - Chambre parent" / "Capteur IAQ - Chambre enfants" / "Capteur IAQ - Salon"
    "device_name": "Capteur IAQ - Chambre parents",

    # Base pour l'ID (le suffixe MAC sera ajouté automatiquement)
    "device_id_base": "iaq_qtpy",

    # période de réveil (secondes)
    "publish_period": 60
}
