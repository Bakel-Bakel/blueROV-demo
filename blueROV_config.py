
ROV_CONFIG = {
    "bluerov1": {
        "host_ip": "192.168.3.10",
        "rov_ip": "192.168.3.11",
        "port": 14551,
        "gripper": False
    },
    "bluerov2": {
        "host_ip": "192.168.3.20",
        "rov_ip": "192.168.3.21",
        "port": 14552,
        "gripper": True,
        "gripper_channel": 7
    }
}

DEFAULT = "bluerov2"

def get_config(rov_name=None):
    if rov_name is None:
        rov_name = DEFAULT
    return ROV_CONFIG.get(rov_name.lower())

