import websockets
import paho.mqtt.client as paho

## MQTT broker
mqtt_broker = 'pumphrey-ie-26'
mqtt_port = 1884
mqtt_topic = "Kepserver/hurco_bust/machine_status"
mqtt_client = paho.Client(paho.CallbackAPIVersion.VERSION1, "PocketNc")


def publish_status(status):
    try:
        mqtt_client.connect(mqtt_broker, mqtt_port)
        mqtt_client.loop_start()
        mqtt_client.publish(mqtt_topic, status)
        print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] Published: Machine is {status}")
    except Exception as e:
        print(f"[MQTT ERROR] Could not publish status: {e}")
    finally:
        mqtt_client.loop_stop()

publish_status("on")