import time
import requests
from pymavlink import mavutil

BACKEND = "https://uav-backend-zbti.onrender.com"
DRONE_ID = "drone_1"
MAVLINK_IN = "udpin:127.0.0.1:14552"

POST_INTERVAL_SEC = 0.5
REQUEST_TIMEOUT = 3.0

state = {
    "lat": None,
    "lng": None,
    "alt_m": 0.0,
    "groundspeed_mps": 0.0,
    "battery_percent": None,
    "mode": None,
    "armed": False,
}

last_post_time = 0.0

def now_ms():
    return int(time.time() * 1000)

def decode_mode(msg):
    try:
        return mavutil.mode_string_v10(msg)
    except Exception:
        return None

def build_payload():
    return {
        "drone_id": DRONE_ID,
        "lat": state["lat"],
        "lng": state["lng"],
        "alt_m": state["alt_m"],
        "groundspeed_mps": state["groundspeed_mps"],
        "battery_percent": state["battery_percent"],
        "mode": state["mode"],
        "armed": state["armed"],
        "ts_ms": now_ms(),
        "source": "ground_relay"
    }

def main():
    global last_post_time

    print(f"[INFO] Listening MAVLink on {MAVLINK_IN}")
    master = mavutil.mavlink_connection(MAVLINK_IN)

    print("[INFO] Waiting heartbeat...")
    master.wait_heartbeat()
    print("[INFO] Heartbeat received")

    while True:
        msg = master.recv_match(blocking=True, timeout=1)
        now = time.time()

        if msg is not None:
            mtype = msg.get_type()

            if mtype == "HEARTBEAT":
                state["armed"] = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                state["mode"] = decode_mode(msg)

            elif mtype == "GLOBAL_POSITION_INT":
                state["lat"] = msg.lat / 1e7
                state["lng"] = msg.lon / 1e7
                state["alt_m"] = msg.relative_alt / 1000.0

            elif mtype == "VFR_HUD":
                state["groundspeed_mps"] = float(msg.groundspeed)

            elif mtype == "SYS_STATUS":
                if getattr(msg, "battery_remaining", -1) not in (-1, 255):
                    state["battery_percent"] = float(msg.battery_remaining)

        can_post = (
            state["lat"] is not None and
            state["lng"] is not None and
            (now - last_post_time) >= POST_INTERVAL_SEC
        )

        if can_post:
            payload = build_payload()
            try:
                r = requests.post(
                    f"{BACKEND}/bridge/telemetry",
                    json=payload,
                    timeout=REQUEST_TIMEOUT
                )
                print(f"[POST] {r.status_code} | {payload['lat']:.6f}, {payload['lng']:.6f} | mode={payload['mode']} | src=ground_relay")
            except Exception as e:
                print(f"[POST ERROR] {e}")

            last_post_time = now

if __name__ == "__main__":
    main()