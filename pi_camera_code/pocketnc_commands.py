# import python libraries
import websockets
import asyncio
import websocket
import _thread
import time
import rel
import json

from datetime import datetime

import paho.mqtt.client as paho

# Connect to MQTT broker
broker = '192.168.1.29'  # IP of the MQTT broker
port = 1883  # Port of the MQTT port
topic = "pocketNc/mqtt/machine/data"  # MQTT topic
topic_mtconnect = "pocketNc/mqtt/mtconnect"  # MQTT topic
topic_pocketnc_data_collection = "pocketNc/mqtt/data"
json_output_dict = {}
last_published_data = {}
client1 = paho.Client(paho.CallbackAPIVersion.VERSION1, "PocketNc")
client1.connect(broker, port)

login_sucessful = False


# Define configuration of the input parameters or data that we want to send
def setCommands():
    commands = [
        "rapidrate",
        "ts",
        "mcodes",
        "estop",
        "velocity",
        "cycle_time",
        "state",
        "actual_position",
        "measured_spindle_speed",
        "command",
        "paused",
        "position",
        "spindle",
        "task_mode",
        "task_state",
        "tool_in_spindle",
        "pressure_data",
        "spindle_pressure",
        "running",
        "program",
        "temperature_data",
        "feedrate",
        "joint_position",
        "gcodes",
        # "tool_table", # not required as per the requirements
        "program_line",  # does not exists
        "program_name"  # does not exists
    ]
    cmd_data = []

    for cmd in commands:
        actual_position = {}
        acmdItem = {"id": cmd, "command": "watch", "name": cmd}
        actual_position = json.dumps(acmdItem)
        cmd_data.append(actual_position)

    return cmd_data


# Send data
async def send_command(ws, command):
    await ws.send(json.dumps(command))


from datetime import datetime
import json

def map_to_mtconnect_metrics(data: dict) -> dict:
    """
    Maps raw PocketNC machine data to MTConnect metric names and formats.
    Each list or nested structure is stored as a single JSON-like field.
    """

    mtconnect_metrics_map = {
        "rapidrate": "RAPID",
        "ts": "timestamp",
        "mcodes": "MCODES",
        "estop": "EMERGENCY_STOP",
        "velocity": "VELOCITY",
        "cycle_time": "CYCLE_TIME",
        "state": "STATE",
        "actual_position": "ACTUAL_POSITION_Y",
        "measured_spindle_speed": "ROTARY_VELOCITY",
        "command": "BLOCK",
        "paused": "EXECUTION",
        "position": "POSITION_Y",
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

    mapped_data = {}

    for key, mt_name in mtconnect_metrics_map.items():
        if key not in data:
            continue

        value = data[key]

        # Convert timestamp to ISO format
        if key == "ts":
            try:
                dt = datetime.strptime(value, "%Y-%m-%d %H:%M:%S")
                mapped_data[mt_name] = dt.isoformat()
            except Exception:
                mapped_data[mt_name] = value

        # Convert binary or state values to human-readable
        elif key == "estop":
            mapped_data[mt_name] = "ARMED" if value == 0 else "TRIGGERED"

        elif key == "task_state":
            mapped_data[mt_name] = "ON" if value == 1 else "OFF"

        elif key == "running":
            mapped_data[mt_name] = "ACTIVE" if value == 1 else "STOPPED"

        elif key == "paused":
            mapped_data[mt_name] = "PAUSED" if value else "RUNNING"

        # Handle list or dict fields as single combined entries
        elif isinstance(value, (list, dict)):
            mapped_data[mt_name] = json.dumps(value)

        # Default direct mapping
        else:
            mapped_data[mt_name] = value

    # Add static or default fields
    mapped_data["Availability"] = "AVAILABLE"
    mapped_data["ROTARY_VELOCITY_OVERRIDE"] = 0

    return mapped_data


# Tranform the message and send/publish data to MQTT topic
async def on_message(ws):
    # global last_published_data

    # receive the data from websocket
    async for message in ws:
        json_object = json.loads(message)
        # print("json object", json_object)
        json_output = {json_object["id"]: json_object["data"]}

        mtconnect_data = {}
        # if 'spindle_pressure' in json_output.keys()  or 'spindle' in json_output.keys():
        akey = list(json_output.keys())[0]
        # if akey in ['spindle_pressure','spindle',"login"] :
        print('data')
        if akey != "login":
            json_output_dict[akey] = json_output[akey]
            json_output_dict["ts"] = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
            # if json_output_dict != last_published_data:
            # json1 = {"data":jso/n_output_dict}

            mtconnect_output = {
                'rapidrate': 'RAPID_RATE',
                'estop': 'EmergencyStop',
                'velocity': 'VELOCITY',
                'ts': 'ClockTime',
                'program': 'Program',
                'state': 'MachineCondition',
                'task_state': 'PowerState',
                'running': 'Execution',
                'measured_spindle_speed': 'ROTARY_VELOCITY',
                'Availability': 'AVAILABLE'
            }
            data = json_output_dict

            # Convert ts to W3C ISO 8601 format (using timezone-aware datetime)

            ts = datetime.strptime(data['ts'], '%Y-%m-%d %H:%M:%S').timestamp()

            # PowerState conversion: 1 ? 'ON', 0 ? 'OFF'
            power_state = "ON" if data.get('task_state') == 1 else "OFF"

            # EmergencyStop conversion: 0 ? 'ARMED', 1 ? 'TRIGGERED'
            estop_state = "ARMED" if data.get('EMERGENCY_STOP', 1) == 0 else "TRIGGERED"

            # Execution state conversion
            execution_map = {
                1: "ACTIVE",  # Actively executing
                0: "STOPPED"  # Not ready to execute
            }
            execution_state = execution_map.get(data.get('running', 0), "STOPPED")

            # Create the final dictionary
            mtconnect_data = {
                mtconnect_output[key]: (
                    ts if key == 'ts' else
                    power_state if key == 'task_state' else
                    estop_state if key == 'estop' else
                    execution_state if key == 'running' else
                    data[key]
                )
                for key in mtconnect_output if key in data
            }
            mtconnect_data['Availability'] = 'AVAILABLE'
            mtconnect_data['ROTARY_VELOCITY_OVERRIDE'] = 0
            print(
                mtconnect_data, mtconnect_data
            )

            # Publish data to mqtt topic
            client1.publish(topic, json.dumps(json_output_dict))
            client1.publish(topic_mtconnect, json.dumps(mtconnect_data))
            # last_published_data = json_output_dict.copy()

            mapped_data = map_to_mtconnect_metrics(json_output_dict)
            client1.publish(topic_pocketnc_data_collection, json.dumps(mapped_data))


def on_error(ws, error):
    print(error)


def on_close(ws, close_status_code, close_msg):
    print("### closed ###")


def on_open(ws):
    print("Opened connection")
    login = json.dumps(
        {
            "id": "login",
            "user": "default",
            "password": "default",
            "ts": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        }
    )
    ws.send(login)
    lxcncCmds = setCommands()

    for acmd in lxcncCmds:
        # if acmd["id"] == 'spindle_pressure':
        #     pass
        # else:
        ws.send(acmd)


async def send_command(ws):
    while True:
        commands = setCommands()
        for command in commands:
            await ws.send(command)
        await asyncio.sleep(60)


# Configuration for connect to websocket
async def connect_and_listen():
    username = 'default'
    password = 'default'
    url = "ws://{username}:{password}@192.168.1.27:8000/websocket/linuxcnc"
    # url =  "ws://{username}:{password}@POCKETNC:8000/websocket/linuxcnc"
    async with websockets.connect(url) as ws:
        login = json.dumps({"id": "login", "user": "default", "password": "default"})
        await ws.send(login)

        await asyncio.gather(on_message(ws), send_command(ws))


if __name__ == "__main__":

    # websocket.enableTrace(True)
    # username = 'default'
    # password = 'default'
    # url =  "ws://{username}:{password}@192.168.1.27:8000/websocket/linuxcnc"
    # # url =  "ws://10.39.0.18:8000/websocket/linuxcnc"
    # ws = websocket.WebSocketApp(url,
    #                           on_open=on_open,
    #                           on_message=on_message,
    #                           on_error=on_error,
    #                           on_close=on_close)

    # ws.run_forever(dispatcher=rel, reconnect=5)  # Set dispatcher to automatic reconnection, 5 second reconnect delay if connection closed unexpectedly
    # rel.signal(2, rel.abort)  # Keyboard Interrupt
    # rel.dispatch();
    try:
        print("program is running")
        asyncio.run(connect_and_listen())
    except KeyboardInterrupt:
        print("program terminated")
