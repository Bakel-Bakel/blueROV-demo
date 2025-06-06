ROV_CONFIG = {
    "bluerov1": {
        "host_ip": "192.168.3.10",
        "rov_ip": "192.168.3.11",
        "port": 14551,
        "gripper": False,
        "channel_map": {
            1: "rotate_y",
            2: "rotate_x",
            3: "move_z",
            4: "rotate_z",
            5: "move_x",
            6: "move_y",
            9: "lights"
        }
    },
    "bluerov2": {
        "host_ip": "192.168.3.20",
        "rov_ip": "192.168.3.21",
        "port": 14552,
        "gripper": True,
        "gripper_channel": 7,
        "channel_map": {
            1: "rotate_y",
            2: "rotate_x",
            3: "move_z",
            4: "rotate_z",
            5: "move_x",
            6: "move_y",
            7: "gripper",
            9: "lights"
        }
    }
}

DEFAULT = "bluerov2"

def get_config(rov_name=None):
    if rov_name is None:
        rov_name = DEFAULT
    return ROV_CONFIG.get(rov_name.lower())
