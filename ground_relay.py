import time
import threading
import queue
import requests
from pymavlink import mavutil

BACKEND = "https://uav-backend-zbti.onrender.com"
DRONE_ID = "drone_1"
MAVLINK_IN = "udpin:127.0.0.1:14552"

POST_INTERVAL_SEC = 2.0
REQUEST_TIMEOUT = (5.0, 5.0)   # (connect_timeout, read_timeout)

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
post_queue = queue.Queue(maxsize=1)


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
        "source": "ground_relay",
    }


def enqueue_latest_payload(payload):
    try:
        post_queue.put_nowait(payload)
    except queue.Full:
        try:
            post_queue.get_nowait()   # bỏ bản cũ
        except queue.Empty:
            pass
        try:
            post_queue.put_nowait(payload)
        except queue.Full:
            pass


def post_worker():
    session = requests.Session()

    while True:
        payload = post_queue.get()
        try:
            r = session.post(
                f"{BACKEND}/bridge/telemetry",
                json=payload,
                timeout=REQUEST_TIMEOUT,
            )
            print(
                f"[POST] {r.status_code} | "
                f"{payload['lat']:.6f}, {payload['lng']:.6f} | "
                f"mode={payload['mode']} | armed={payload['armed']}"
            )
        except Exception as e:
            print(f"[POST ERROR] {e}")


def main():
    global last_post_time

    print(f"[INFO] Listening MAVLink on {MAVLINK_IN}")
    master = mavutil.mavlink_connection(MAVLINK_IN)

    print("[INFO] Waiting heartbeat...")
    hb = master.wait_heartbeat()
    fc_sys = hb.get_srcSystem()
    fc_comp = hb.get_srcComponent()
    print(f"[INFO] Heartbeat received from sys={fc_sys}, comp={fc_comp}")

    threading.Thread(target=post_worker, daemon=True).start()

    last_mode = None
    last_armed = None

    while True:
        msg = master.recv_match(blocking=True, timeout=0.1)
        now = time.monotonic()

        if msg is not None:
            src_sys = msg.get_srcSystem()
            src_comp = msg.get_srcComponent()

            if src_sys == fc_sys and src_comp == fc_comp:
                mtype = msg.get_type()

                if mtype == "HEARTBEAT":
                    new_armed = bool(
                        msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    new_mode = decode_mode(msg)

                    state["armed"] = new_armed
                    state["mode"] = new_mode

                    if new_mode != last_mode or new_armed != last_armed:
                        print(f"[HB] mode={new_mode} armed={new_armed}")
                        last_mode = new_mode
                        last_armed = new_armed

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
            state["lat"] is not None
            and state["lng"] is not None
            and (now - last_post_time) >= POST_INTERVAL_SEC
        )

        if can_post:
            enqueue_latest_payload(build_payload())
            last_post_time = now


if __name__ == "__main__":
    main()