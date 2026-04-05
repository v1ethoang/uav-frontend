import time
import math
import threading
import socketio
from pymavlink import mavutil

# =========================================================
# CẤU HÌNH HỆ THỐNG
# =========================================================
BACKEND = "http://localhost:8000"  # Đổi thành URL Server của bạn khi bay thật
DRONE_ID = "drone_1"
MAVLINK_IN = "udpin:127.0.0.1:14552"

# Tọa độ kho (Giống hệt trong file bridge.py)
WAREHOUSE_LAT = 10.9327877
WAREHOUSE_LNG = 106.7669519

TELEMETRY_INTERVAL = 0.2  # 5Hz - Giúp bản đồ mượt như app gọi xe
CONNECT_TIMEOUT = 5

sio = socketio.Client(reconnection=True, reconnection_attempts=0, reconnection_delay=2)

# =========================================================
# QUẢN LÝ TRẠNG THÁI
# =========================================================
state = {
    "lat": None,
    "lng": None,
    "alt_m": 0.0,
    "groundspeed_mps": 0.0,
    "battery_percent": None,
    "mode": None,
    "armed": False,
}

# Các cờ (flag) để Ground Station tự suy luận event khi Pi mất mạng
mission_tracker = {
    "is_flying": False,
    "arrived_sent": False,
    "delivered_sent": False,
    "completed_sent": False,
}

def now_ms():
    return int(time.time() * 1000)

def decode_mode(msg):
    try:
        return mavutil.mode_string_v10(msg)
    except Exception:
        return None

def haversine(lat1, lon1, lat2, lon2):
    if lat1 is None or lon1 is None: return 999999.0
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2)**2
    return 2 * R * math.asin(math.sqrt(a))

# =========================================================
# SOCKET.IO EVENTS
# =========================================================
@sio.event
def connect():
    print(f"[SOCKET] ⚡ Đã kết nối thành công tới Backend: {BACKEND}")

@sio.event
def disconnect():
    print("[SOCKET] ⚠️ Mất kết nối tới Backend, đang thử lại...")

def emit_event(event_type, detail=None):
    """Hàm gửi Event thay thế cho Pi khi mất wifi"""
    if not sio.connected:
        print(f"[EVENT] Không thể gửi {event_type} do mất mạng Server.")
        return

    payload = {
        "drone_id": DRONE_ID,
        "mission_id": "", # Trạm mặt đất không biết chính xác ID, Backend sẽ tự map
        "type": event_type,
        "detail": detail,
        "ts_ms": now_ms()
    }
    try:
        sio.emit('bridge_event', payload)
        print(f"[EVENT] -> {event_type} (Gửi bởi Ground Relay)")
    except Exception as e:
        print(f"[EVENT ERROR] {e}")

# =========================================================
# VÒNG LẶP SUY LUẬN NHIỆM VỤ (HEURISTIC LOGIC)
# =========================================================
def check_mission_events():
    """Tự động phân tích telemetry để bắn event DELIVERED / COMPLETED"""
    global mission_tracker

    if state["lat"] is None or state["lng"] is None:
        return

    dist_to_home = haversine(state["lat"], state["lng"], WAREHOUSE_LAT, WAREHOUSE_LNG)
    alt = state["alt_m"]
    speed = state["groundspeed_mps"]

    # 1. Phát hiện cất cánh (Bắt đầu một chuyến bay mới)
    if state["armed"] and alt > 2.0 and not mission_tracker["is_flying"]:
        mission_tracker["is_flying"] = True
        mission_tracker["arrived_sent"] = False
        mission_tracker["delivered_sent"] = False
        mission_tracker["completed_sent"] = False
        print("[TRACKER] Drone đã cất cánh, bắt đầu giám sát chuyến bay.")

    if not mission_tracker["is_flying"]:
        return

    # 2. Phát hiện đến điểm giao hàng (ARRIVED) & Đang thả hàng (DELIVERED)
    # Điều kiện: Cách nhà > 5m, tốc độ giảm (< 1.0 m/s), độ cao hạ xuống (< 7m)
    if dist_to_home > 5.0 and speed < 1.0:
        if alt < 8.0 and not mission_tracker["arrived_sent"]:
            emit_event("ARRIVED", "Đã đến tọa độ giao, đang hạ độ cao")
            mission_tracker["arrived_sent"] = True
            
        if alt < 6.5 and mission_tracker["arrived_sent"] and not mission_tracker["delivered_sent"]:
            emit_event("DELIVERED", "Đã thả hàng thành công (Xác nhận từ Relay)")
            mission_tracker["delivered_sent"] = True

    # 3. Phát hiện bay về kho thành công (COMPLETED)
    # Điều kiện: Cách nhà < 15m, đang ở mode LAND hoặc đã Disarm
    if dist_to_home < 15.0 and (state["mode"] == "LAND" or not state["armed"]):
        if alt < 2.0 and not mission_tracker["completed_sent"]:
            emit_event("COMPLETED", "Đã hạ cánh tại kho (Xác nhận từ Relay)")
            mission_tracker["completed_sent"] = True
            mission_tracker["is_flying"] = False # Kết thúc chuyến bay

# =========================================================
# VÒNG LẶP GỬI TELEMETRY (CHẠY NGẦM)
# =========================================================
def telemetry_worker():
    while True:
        if sio.connected and state["lat"] is not None and state["lng"] is not None:
            payload = {
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
            try:
                sio.emit('bridge_telemetry', payload)
            except Exception:
                pass
        
        # Gửi cực mượt với tốc độ được định nghĩa ở trên (VD: 0.2s = 5 lần/giây)
        time.sleep(TELEMETRY_INTERVAL) 

# =========================================================
# LUỒNG CHÍNH ĐỌC MAVLINK
# =========================================================
def main():
    print(f"[INFO] Kết nối MAVLink tại {MAVLINK_IN}")
    master = mavutil.mavlink_connection(MAVLINK_IN)

    print("[INFO] Đang chờ Heartbeat từ Flight Controller...")
    hb = master.wait_heartbeat()
    fc_sys = hb.get_srcSystem()
    fc_comp = hb.get_srcComponent()
    print(f"[INFO] Đã nhận Heartbeat: sys={fc_sys}, comp={fc_comp}")

    # Chạy kết nối Socket.io ở chế độ nền
    try:
        print("[INFO] Đang kết nối tới Backend...")
        sio.connect(BACKEND)
    except Exception as e:
        print(f"[WARNING] Chưa kết nối được Backend ngay lập tức, sẽ thử lại ngầm. Lỗi: {e}")

    # Chạy thread gửi Telemetry
    threading.Thread(target=telemetry_worker, daemon=True).start()

    last_mode = None
    last_armed = None

    while True:
        msg = master.recv_match(blocking=True, timeout=0.1)
        if msg is not None:
            src_sys = msg.get_srcSystem()
            src_comp = msg.get_srcComponent()

            if src_sys == fc_sys and src_comp == fc_comp:
                mtype = msg.get_type()

                if mtype == "HEARTBEAT":
                    new_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    new_mode = decode_mode(msg)

                    state["armed"] = new_armed
                    state["mode"] = new_mode

                    if new_mode != last_mode or new_armed != last_armed:
                        print(f"[FC] mode={new_mode} | armed={new_armed}")
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

        # Kiểm tra và tự động bắn event nếu thỏa mãn logic
        check_mission_events()

if __name__ == "__main__":
    main()