import asyncio
import websockets
import json
import time
from datetime import datetime
import logging
import paho.mqtt.client as mqtt

# ---------------- CONFIGURE LOGGING ----------------
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s', 
    datefmt='%Y-%m-%d %H:%M:%S'
)

# ---------------- MQTT CONFIG ----------------
broker = '192.168.1.8'
port = 1883
topic_raw = "pocketNc/mqtt/machine/data"
topic_mtconnect = "pocketNc/mqtt/mtconnect"
topic_collection = "pocketNc/mqtt/data"

mqtt_client = mqtt.Client(
    client_id="PocketNc",
    protocol=mqtt.MQTTv311,
    callback_api_version=mqtt.CallbackAPIVersion.VERSION2
)

mqtt_client.connect(broker, port)
mqtt_client.loop_start()
logging.info("Connected to MQTT broker at %s:%s", broker, port)

# ---------------- GLOBAL STATE ----------------
json_output_dict = {}

# ---------------- COMMANDS ----------------
def setCommands():
    commands = [
        "rapidrate","ts","mcodes","estop","velocity","cycle_time",
        "state","actual_position","measured_spindle_speed","command",
        "paused","position","spindle","task_mode","task_state",
        "tool_in_spindle","pressure_data","spindle_pressure","running",
        "program","temperature_data","feedrate","joint_position","gcodes"
    ]
    return [json.dumps({"id": cmd, "command": "watch", "name": cmd}) for cmd in commands]

# ---------------- MTConnect MAPPING ----------------
def map_to_mtconnect_metrics(data: dict) -> dict:
    mapped_data = {}
    mapping = {
        "rapidrate": "RAPID",
        "ts": "timestamp",
        "mcodes": "MCODES",
        "estop": "EMERGENCY_STOP",
        "velocity": "VELOCITY",
        "cycle_time": "CYCLE_TIME",
        "state": "STATE",
        "actual_position": "ACTUAL_POSITION",
        "measured_spindle_speed": "ROTARY_VELOCITY",
        "command": "BLOCK",
        "paused": "EXECUTION",
        "position": "POSITION",
        "spindle": "SPINDLE_DATA",
        "task_mode": "TASK_MODE",
        "task_state": "TASK_STATE",
        "tool_in_spindle": "TOOL",
        "pressure_data": "PRESSURE",
        "spindle_pressure": "SPINDLE_PRESSURE",
        "running": "RUNNING",
        "program": "PROGRAM",
        "temperature_data": "TEMPERATURE",
        "feedrate": "FEEDRATE",
        "joint_position": "JOINT_POSITION",
        "gcodes": "GCODES"
    }

    for key, mt_name in mapping.items():
        if key not in data:
            continue
        value = data[key]

        if key == "ts":
            try:
                dt = datetime.strptime(value, "%Y-%m-%d %H:%M:%S")
                mapped_data[mt_name] = dt.isoformat()
            except Exception:
                mapped_data[mt_name] = value
        elif key == "estop":
            mapped_data[mt_name] = "ARMED" if value == 0 else "TRIGGERED"
        elif key == "task_state":
            mapped_data[mt_name] = "ON" if value == 1 else "OFF"
        elif key == "running":
            mapped_data[mt_name] = "ACTIVE" if value == 1 else "STOPPED"
        elif key == "paused":
            mapped_data[mt_name] = "PAUSED" if value else "RUNNING"
        elif isinstance(value, (list, dict)):
            mapped_data[mt_name] = json.dumps(value)
        else:
            mapped_data[mt_name] = value

    mapped_data["Availability"] = "AVAILABLE"
    mapped_data["ROTARY_VELOCITY_OVERRIDE"] = 0
    return mapped_data

# ---------------- SEND COMMANDS ----------------
async def send_commands_periodically(ws):
    while True:
        commands = setCommands()
        for cmd in commands:
            await ws.send(cmd)
            logging.debug("Sent command: %s", cmd)
        await asyncio.sleep(5)  # send commands every 5 seconds

# ---------------- RECEIVE AND PUBLISH ----------------
async def receive_and_publish(ws):
    async for message in ws:
        try:
            msg = json.loads(message)
        except Exception as e:
            logging.warning("Failed to parse message: %s", e)
            continue

        key = msg.get("id")
        if key and key != "login":
            json_output_dict[key] = msg.get("data")
            json_output_dict["ts"] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # Publish raw data
            mqtt_client.publish(topic_raw, json.dumps(json_output_dict))
            logging.info("Published raw data to %s", topic_raw)

            # Publish MTConnect data
            mtconnect_data = {
                "RAPID": json_output_dict.get("rapidrate"),
                "ClockTime": json_output_dict.get("ts"),
                "EMERGENCY_STOP": "ARMED" if json_output_dict.get("estop") == 0 else "TRIGGERED",
                "Execution": "ACTIVE" if json_output_dict.get("running") == 1 else "STOPPED",
                "Availability": "AVAILABLE",
                "ROTARY_VELOCITY_OVERRIDE": 0
            }
            mqtt_client.publish(topic_mtconnect, json.dumps(mtconnect_data))
            logging.info("Published MTConnect data to %s", topic_mtconnect)

            # Publish mapped data
            mapped_data = map_to_mtconnect_metrics(json_output_dict)
            mqtt_client.publish(topic_collection, json.dumps(mapped_data))
            logging.info("Published mapped data to %s", topic_collection)

# ---------------- CONNECT AND RUN ----------------
async def connect_and_listen():
    url = "ws://default:default@192.168.1.27:8000/websocket/linuxcnc"

    async with websockets.connect(url) as ws:
        logging.info("Connected to WebSocket: %s", url)
        # Login
        await ws.send(json.dumps({"id": "login", "user": "default", "password": "default"}))
        logging.info("Sent login command")

        # Run send & receive concurrently
        await asyncio.gather(
            send_commands_periodically(ws),
            receive_and_publish(ws)
        )

# ---------------- MAIN ----------------
if __name__ == "__main__":
    logging.info("Program started, connecting to WebSocket...")
    try:
        asyncio.run(connect_and_listen())
    except KeyboardInterrupt:
        logging.info("Program terminated by user")
