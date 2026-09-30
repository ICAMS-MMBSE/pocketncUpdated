
import asyncio
import websockets
import paho.mqtt.client as paho
import time

## Kep Node MQTT broker
mqtt_broker = '192.168.1.29'
mqtt_port = 1883
mqtt_topic = "pocketNc/mqtt/machine/status"
mqtt_client = paho.Client(paho.CallbackAPIVersion.VERSION1, "PocketNc")

## PocketNC Websocket config and credentials
username = 'default'
password = 'default'
url =  "ws://{username}:{password}@192.168.1.27:8000/websocket/linuxcnc"

# Publish machine running status on MQTT topic
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
        mqtt_client.disconnect()

# Check machine running status
async def check_machine_status(interval=5):
    print("checking machine running status")
    while True:
        await asyncio.sleep(interval)
        try:
            async with websockets.connect(url) as websocket:
                publish_status("on")
                print("on")
        except Exception as e:
            publish_status("off")
            print("off")
            print(f"[WebSocket ERROR] Machine is OFF | {e}")


# Run the machine status check
if __name__ == "__main__":
    try:
        asyncio.run(check_machine_status())
    except KeyboardInterrupt:
        print("\nStopped by user.")
