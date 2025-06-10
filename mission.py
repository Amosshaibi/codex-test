import socket
import sys
print("🧭 مسار النظام:", sys.path)
print("🧭 dronekit من:", __import__('dronekit').__file__)
print("🧭 pymavlink من:", __import__('pymavlink').__file__)
import os
import json
import datetime
import time
import math
import threading
import requests
from flask import Flask, send_from_directory, request, jsonify
from flask_cors import CORS

try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative
    from pymavlink import mavutil
    DRONE_CONNECTIVITY_ENABLED = True
    print("[معلومات] تم تحميل DroneKit و Pymavlink بنجاح.")
except Exception as e:
    DRONE_CONNECTIVITY_ENABLED = False
    print(f"[تحذير] لم يتم العثور على DroneKit/Pymavlink: {e}. تم تعطيل ميزات اتصال الدرون.")

# --- الحالة العامة والإعدادات ---
mission_cancel_flag = False
mission_cancel_lock = threading.Lock() 
current_drone_location = {"lat": 0.0, "lon": 0.0, "alt": 0.0, "mode": "غير معروف"}
drone_location_lock = threading.Lock()
SERVER_PORT = 5055
BASE_DIR = os.path.join(os.path.dirname(__file__), "www")
POINTS_FILE = os.path.join(BASE_DIR, "mission_data.json")
LOG_FILE = os.path.join(BASE_DIR, "mission_log.txt")
BACKUP_FILE = os.path.join(BASE_DIR, "mission_data_backup.json")

# إعدادات اتصال الدرون (قابلة للتخصيص عبر متغيرات البيئة)
DRONE_CONNECTION_STRING = os.getenv('DRONE_CONNECTION_STRING', '/dev/ttyACM0')
DRONE_BAUD_RATE = int(os.getenv('DRONE_BAUD_RATE', '57600'))
DRONE_CONNECTION_TIMEOUT = 60  # ثواني

app = Flask(__name__, static_folder=BASE_DIR)
CORS(app)

def log_event(message, level="معلومات"):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_entry = f"[{timestamp}] [{level}] {message}\n"
    try:
        with open(LOG_FILE, "a", encoding='utf-8') as f:
            f.write(log_entry)
    except Exception as e:
        print(f"[خطأ] فشل الكتابة في ملف السجل: {e}")
    print(log_entry.strip())

def check_mission_cancel_flag():
    with mission_cancel_lock:
        return mission_cancel_flag

def set_mission_cancel_flag(value):
    with mission_cancel_lock:
        global mission_cancel_flag
        mission_cancel_flag = value

def get_distance_meters(location1, location2):
    R = 6371e3
    lat1 = math.radians(location1.lat)
    lat2 = math.radians(location2.lat)
    delta_lat = math.radians(location2.lat - location1.lat)
    delta_lon = math.radians(location2.lon - location1.lon)
    
    a = math.sin(delta_lat/2) * math.sin(delta_lat/2) + \
        math.cos(lat1) * math.cos(lat2) * \
        math.sin(delta_lon/2) * math.sin(delta_lon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c

def send_servo_command(vehicle, servo_num, pwm_value):
    if not DRONE_CONNECTIVITY_ENABLED: 
        return
        
    try:
        msg = vehicle.message_factory.command_long_encode(
            0, 0, 
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 
            0,
            servo_num,
            pwm_value,
            0, 0, 0, 0, 0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        log_event(f"تم إرسال أمر سيرفو للقناة {servo_num} بقيمة PWM: {pwm_value}", "معلومات")
    except Exception as e:
        log_event(f"فشل إرسال أمر سيرفو للقناة {servo_num}: {e}", "خطأ")

def update_drone_location():
    """خلفية لتحديث موقع الدرون باستمرار"""
    vehicle = None
    while True:
        try:
            if DRONE_CONNECTIVITY_ENABLED and not vehicle:
                try:
                    vehicle = connect(
                        DRONE_CONNECTION_STRING, 
                        wait_ready=False, 
                        baud=DRONE_BAUD_RATE, 
                        timeout=10
                    )
                    log_event("تم إنشاء اتصال خلفي لتحديثات الموقع", "معلومات")
                except Exception as e:
                    log_event(f"فشل الاتصال الخلفي: {e}", "خطأ")
                    vehicle = None
                    time.sleep(5)
                    continue
            
            if vehicle and vehicle.location.global_relative_frame:
                with drone_location_lock:
                    loc = vehicle.location.global_relative_frame
                    current_drone_location.update({
                        "lat": loc.lat,
                        "lon": loc.lon,
                        "alt": loc.alt,
                        "mode": vehicle.mode.name if vehicle.mode else "غير معروف"
                    })
                    print(f"[تصحيح] تحديث موقع الدرون: خط العرض: {loc.lat:.6f}, خط الطول: {loc.lon:.6f}, الارتفاع: {loc.alt:.1f}m")
        except Exception as e:
            log_event(f"خطأ في تحديث الموقع: {e}", "خطأ")
            vehicle = None
        finally:
            time.sleep(1)

def execute_drone_mission(mission_points, rtl_options):
    log_event("========== بدء تنفيذ المهمة ==========", "معلومات")
    vehicle = None
    active_servo_channels = {}  # تتبع أوامر السيرفو النشطة
    
    try:
        connection_string = DRONE_CONNECTION_STRING
        baud = DRONE_BAUD_RATE
        
        for attempt in range(1, 6):  # حاول حتى 5 مرات
            if check_mission_cancel_flag(): 
                log_event("تم إلغاء المهمة أثناء محاولة الاتصال.", "تحذير")
                return
                
            log_event(f"محاولة الاتصال بالدرون (المحاولة {attempt}/5)...", "معلومات")
            try:
                vehicle = connect(
                    connection_string, 
                    wait_ready=True, 
                    baud=baud, 
                    timeout=DRONE_CONNECTION_TIMEOUT
                )
                log_event("تم الاتصال بالدرون بنجاح.", "نجاح")
                break
            except Exception as e:
                log_event(f"فشلت محاولة الاتصال {attempt}: {e}", "خطأ")
                vehicle = None
                time.sleep(3)
        
        if not vehicle:
            log_event("فشل الاتصال بالدرون بعد 5 محاولات. إيقاف المهمة.", "حرج")
            return

        log_event("التحقق من جاهزية الدرون للتشغيل...", "معلومات")

        unarmable_start_time = time.time()
        while not vehicle.is_armable:
            if check_mission_cancel_flag():
                log_event("تم إلغاء المهمة أثناء انتظار الجاهزية.", "تحذير")
                return

            elapsed = time.time() - unarmable_start_time

            try:
                reason = []
                if hasattr(vehicle, "system_status") and vehicle.system_status:
                    reason.append(f"الحالة: {vehicle.system_status.state}")
                if hasattr(vehicle, "gps_0") and vehicle.gps_0:
                    reason.append(f"GPS Fix: {vehicle.gps_0.fix_type}")
                if hasattr(vehicle, "ekf_ok"):
                    reason.append("EKF: ✅" if vehicle.ekf_ok else "EKF: ❌")
                if hasattr(vehicle, "battery") and vehicle.battery and vehicle.battery.voltage:
                    reason.append(f"بطارية: {vehicle.battery.voltage:.1f}V")

                reason_str = " | ".join(reason) if reason else "سبب غير معروف"
                log_event(f"🚧 الدرون غير جاهز بعد ({int(elapsed)} ث). {reason_str}", "تحذير")

            except Exception as e:
                log_event(f"🚧 الدرون غير جاهز بعد ({int(elapsed)} ث). (فشل في تحليل السبب: {e})", "تحذير")

            if elapsed >= 120:
                log_event("❌ لم يصبح الدرون جاهزًا خلال دقيقتين. تم إيقاف المهمة.", "حرج")
                return

            time.sleep(2)

        
        log_event("تغيير الوضع إلى GUIDED.", "معلومات")
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.flush()
        
        log_event("تشغيل المحركات...", "معلومات")
        vehicle.armed = True

        arm_timeout = 0
        while not vehicle.armed:
            if check_mission_cancel_flag(): 
                log_event("تم إلغاء المهمة أثناء التشغيل.", "تحذير")
                return
                
            log_event(f"في انتظار التشغيل... {arm_timeout+1} ثواني", "تصحيح")
            time.sleep(1)
            arm_timeout += 1
            if arm_timeout > 30:
                log_event("فشل التشغيل خلال 30 ثانية. إيقاف المهمة.", "حرج")
                return
        
        log_event("تم تشغيل المحركات بنجاح.", "نجاح")

        first_point = mission_points[0]
        takeoff_alt = first_point.get("alt", 20)
        log_event(f"بدء الإقلاع العمودي إلى ارتفاع {takeoff_alt} متر.", "معلومات")
        vehicle.simple_takeoff(takeoff_alt)
        
        while True:
            if check_mission_cancel_flag(): 
                log_event("تم إلغاء المهمة أثناء الإقلاع.", "تحذير")
                break
                
            current_alt = vehicle.location.global_relative_frame.alt
            if current_alt is None:
                time.sleep(1)
                continue
                
            if current_alt >= takeoff_alt * 0.95:
                log_event(f"تم الوصول إلى ارتفاع الإقلاع: {current_alt:.1f} متر.", "معلومات")
                break
                
            log_event(f"الارتفاع الحالي: {current_alt:.1f} متر...", "تصحيح")
            time.sleep(1)

        for i, point in enumerate(mission_points):
            if check_mission_cancel_flag(): 
                log_event("تم إلغاء المهمة أثناء التنقل بين النقاط.", "تحذير")
                break
                
            log_event(f"--- الانتقال إلى النقطة {i+1}: '{point.get('name', 'غير معروف')}' ---", "معلومات")
            target_loc = LocationGlobalRelative(point['lat'], point['lon'], point['alt'])
            vehicle.simple_goto(target_loc)

            while True:
                if check_mission_cancel_flag(): 
                    log_event("تم إلغاء المهمة أثناء الحركة.", "تحذير")
                    break
                    
                current_loc = vehicle.location.global_relative_frame
                if current_loc is None:
                    time.sleep(1)
                    continue
                    
                dist = get_distance_meters(current_loc, target_loc)
                log_event(f"المسافة إلى النقطة {i+1}: {dist:.1f} متر.", "تصحيح")
                if dist <= 1.0:
                    log_event(f"تم الوصول إلى النقطة {i+1}.", "نجاح")
                    break
                    
                time.sleep(1)
            
            if check_mission_cancel_flag():
                break

            # انتظار قبل الإجراء
            wait_before = point.get('wait_before', 0)
            if wait_before > 0:
                log_event(f"انتظار قبل الإجراءات لمدة {wait_before} ثواني.", "معلومات")
                start_time = time.time()
                while time.time() - start_time < wait_before:
                    if check_mission_cancel_flag(): 
                        log_event("تم إلغاء المهمة أثناء الانتظار.", "تحذير")
                        break
                    time.sleep(0.5)
                
                if check_mission_cancel_flag():
                    break

            # إجراءات السيرفو
            if point.get('servo', False):
                log_event("تفعيل سيرفو القناة 9.", "معلومات")
                send_servo_command(vehicle, 9, 1750)
                active_servo_channels[9] = (1750, time.time())
                time.sleep(1)
                send_servo_command(vehicle, 9, 1500)
                active_servo_channels[9] = (1500, time.time())
            
            channel = point.get('channel', 'OFF')
            if channel != 'OFF' and not check_mission_cancel_flag():
                channel_num = int(channel)
                pwm = point.get('pwm', 1900)
                duration = point.get('duration', 0)
                log_event(f"تفعيل سيرفو القناة {channel_num} بقيمة PWM {pwm} لمدة {duration} ثواني.", "معلومات")
                send_servo_command(vehicle, channel_num, pwm)
                active_servo_channels[channel_num] = (pwm, time.time())
                
                start_time = time.time()
                while time.time() - start_time < duration:
                    if check_mission_cancel_flag():
                        break
                    time.sleep(0.1)
                
                # العودة إلى الوضع المحايد إذا لم يتم الإلغاء
                if not check_mission_cancel_flag():
                    send_servo_command(vehicle, channel_num, 1500)
                    active_servo_channels[channel_num] = (1500, time.time())
                    log_event(f"تم إرجاع القناة {channel_num} إلى الوضع المحايد.", "معلومات")

            if check_mission_cancel_flag():
                break

            # انتظار بعد الإجراء
            wait_after = point.get('wait_after', 0)
            if wait_after > 0:
                log_event(f"انتظار بعد الإجراءات لمدة {wait_after} ثواني.", "معلومات")
                start_time = time.time()
                while time.time() - start_time < wait_after:
                    if check_mission_cancel_flag(): 
                        log_event("تم إلغاء المهمة أثناء الانتظار.", "تحذير")
                        break
                    time.sleep(0.5)
                
                if check_mission_cancel_flag():
                    break
        
        # إعادة تعيين جميع السيرفو النشطة إذا تم إلغاء المهمة
        if check_mission_cancel_flag():
            log_event("إعادة تعيين السيرفو النشط إلى الوضع المحايد...", "تحذير")
            for channel, (_, _) in active_servo_channels.items():
                try:
                    send_servo_command(vehicle, channel, 1500)
                    log_event(f"تم إعادة تعيين القناة {channel} إلى الوضع المحايد", "معلومات")
                except Exception as e:
                    log_event(f"فشل إعادة تعيين القناة {channel}: {e}", "خطأ")
        
        # العودة إلى نقطة الإطلاق أو نقطة مخصصة
        if check_mission_cancel_flag():
            log_event("تم إلغاء المهمة. العودة إلى نقطة الإطلاق...", "تحذير")
            vehicle.mode = VehicleMode("RTL")
        else:
            log_event("تم إكمال جميع النقاط. بدء العودة...", "معلومات")
            if rtl_options['mode'] == 'rtl':
                log_event("تفعيل وضع RTL.", "معلومات")
                vehicle.mode = VehicleMode("RTL")
            else:  # نقطة عودة مخصصة
                log_event(f"التوجه إلى نقطة العودة المخصصة: {rtl_options['lat']}, {rtl_options['lon']} على ارتفاع {rtl_options['alt']} متر.", "معلومات")
                custom_rtl_loc = LocationGlobalRelative(rtl_options['lat'], rtl_options['lon'], rtl_options['alt'])
                vehicle.simple_goto(custom_rtl_loc)
                
                # الانتظار حتى الوصول إلى النقطة المخصصة
                while not check_mission_cancel_flag():
                    current_loc = vehicle.location.global_relative_frame
                    if current_loc is None:
                        time.sleep(1)
                        continue
                        
                    dist = get_distance_meters(current_loc, custom_rtl_loc)
                    log_event(f"المسافة إلى نقطة العودة: {dist:.1f} متر.", "تصحيح")
                    if dist <= 1.0:
                        log_event("تم الوصول إلى نقطة العودة المخصصة. الهبوط.", "معلومات")
                        vehicle.mode = VehicleMode("LAND")
                        break
                    time.sleep(1)
        
        # الانتظار للهبوط وإيقاف التشغيل
        while vehicle.armed and not check_mission_cancel_flag():
            time.sleep(2)
            if vehicle.location.global_relative_frame:
                log_event(f"الارتفاع الحالي: {vehicle.location.global_relative_frame.alt:.1f} متر", "تصحيح")
            
        if check_mission_cancel_flag():
            log_event("تم إلغاء المهمة أثناء الهبوط.", "تحذير")
        else:
            log_event("هبط الدرون وتم إيقاف التشغيل.", "نجاح")

    except Exception as e:
        log_event(f"خطأ حرج في المهمة: {e}", "حرج")
        if vehicle and vehicle.armed:
            try:
                log_event("تفعيل وضع RTL.", "تحذير")
                vehicle.mode = VehicleMode("RTL")
            except:
                log_event("فشل تفعيل وضع RTL", "خطأ")
    finally:
        if vehicle:
            try:
                log_event("قطع الاتصال بالدرون.", "معلومات")
                vehicle.close()
            except:
                log_event("خطأ في قطع الاتصال بالدرون", "خطأ")
        log_event("========== انتهت المهمة ==========", "معلومات")
        set_mission_cancel_flag(False)  # إعادة تعيين العلم للمهمة التالية

# --- مسارات Flask ---
@app.route('/api/elevation')
def api_get_elevation():
    lat = request.args.get('lat'); lon = request.args.get('lon')
    try:
        url = f"https://api.open-meteo.com/v1/elevation?latitude={lat}&longitude={lon}"
        response = requests.get(url, timeout=10)
        response.raise_for_status()
        elevation = float(response.json()["elevation"][0])
        return jsonify({"elevation": elevation})
    except Exception as e:
        log_event(f"خطأ في API الارتفاع: {e}", "خطأ")
        return jsonify({"error": "فشل الحصول على الارتفاع"}), 500

@app.after_request
def add_cors_headers(r):
    r.headers['Access-Control-Allow-Origin'] = '*'
    r.headers['Access-Control-Allow-Headers'] = 'Content-Type'
    r.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS, DELETE'
    return r

@app.route('/')
def serve_index(): 
    return send_from_directory(BASE_DIR, "index.html")

@app.route('/api/status')
def get_server_status():
    return jsonify({
        "connected": True, 
        "drone_connectivity_enabled": DRONE_CONNECTIVITY_ENABLED,
        "drone_connection_string": DRONE_CONNECTION_STRING,
        "drone_baud_rate": DRONE_BAUD_RATE
    })

@app.route('/api/mission/save', methods=['POST'])
def api_save_mission():
    try:
        data = request.json
        if not data: return jsonify({"status": "بيانات غير صالحة"}), 400
        if os.path.exists(POINTS_FILE): 
            os.replace(POINTS_FILE, BACKUP_FILE)
        with open(POINTS_FILE, 'w', encoding='utf-8') as f: 
            json.dump(data, f, indent=4, ensure_ascii=False)
        log_event("تم حفظ بيانات المهمة بنجاح.", "نجاح")
        return jsonify({"status": "تم الحفظ بنجاح."})
    except Exception as e:
        log_event(f"فشل حفظ المهمة: {e}", "خطأ")
        return jsonify({"status": f"فشل الحفظ: {e}"}), 500

@app.route('/api/mission/load', methods=['GET'])
def api_load_mission():
    try:
        file_to_load = POINTS_FILE if os.path.exists(POINTS_FILE) else BACKUP_FILE
        if os.path.exists(file_to_load):
            with open(file_to_load, 'r', encoding='utf-8') as f:
                data = json.load(f)
                log_event("تم تحميل بيانات المهمة.", "نجاح")
                return jsonify(data)
        log_event("لم يتم العثور على ملف مهمة، جاري إنشاء ملف افتراضي.", "تحذير")
        return jsonify({"points": [], "rtl": {"mode": "rtl", "alt": 20, "lat": 0, "lon": 0}})
    except Exception as e:
        log_event(f"فشل تحميل المهمة: {e}", "خطأ")
        return jsonify({"status": f"فشل التحميل: {e}"}), 500

@app.route('/api/log', methods=['GET', 'DELETE'])
def api_mission_log():
    if request.method == 'GET':
        if not os.path.exists(LOG_FILE): 
            return ""
        with open(LOG_FILE, 'r', encoding='utf-8') as f: 
            return f.read()
    elif request.method == 'DELETE':
        open(LOG_FILE, 'w').close()
        log_event("تم مسح سجل المهمة.", "معلومات")
        return jsonify({"status": "تم مسح السجل."})

@app.route('/api/drone/location', methods=['GET'])
def api_drone_location():
    with drone_location_lock: 
        return jsonify(current_drone_location)

@app.route('/api/mission/start', methods=['POST'])
def api_start_mission():
    if not DRONE_CONNECTIVITY_ENABLED:
        log_event("فشل بدء المهمة: اتصال الدرون معطل.", "خطأ")
        return jsonify({"status": "فشل بدء المهمة: اتصال الدرون معطل."}), 503
    
    set_mission_cancel_flag(False)
    try:
        with open(POINTS_FILE, 'r', encoding='utf-8') as f: 
            mission_data = json.load(f)
            
        points_to_execute = [p for p in mission_data.get("points", []) if p.get("enabled", True)]
        if not points_to_execute:
            log_event("فشل بدء المهمة: لا توجد نقاط مفعلة.", "تحذير")
            return jsonify({"status": "لا توجد نقاط مفعلة لبدء المهمة."}), 400
            
        rtl_options = mission_data.get("rtl", {"mode": "rtl", "alt": 20, "lat": 0, "lon": 0})
        
        mission_thread = threading.Thread(
            target=execute_drone_mission, 
            args=(points_to_execute, rtl_options)
        )
        mission_thread.daemon = True
        mission_thread.start()
        
        log_event("تم إرسال أمر بدء المهمة.", "نجاح")
        return jsonify({"status": "بدأ تنفيذ المهمة."})
    except Exception as e:
        log_event(f"فشل بدء المهمة بسبب خطأ في البيانات: {e}", "خطأ")
        return jsonify({"status": f"فشل تحميل البيانات: {e}"}), 500

@app.route('/api/mission/end', methods=['POST'])
def api_end_mission():
    set_mission_cancel_flag(True)
    log_event("تم إرسال أمر إنهاء المهمة.", "تحذير")
    return jsonify({"status": "تم طلب إنهاء المهمة."})

if __name__ == '__main__':
    if not os.path.exists(BASE_DIR): 
        os.makedirs(BASE_DIR)
    if not os.path.exists(LOG_FILE): 
        open(LOG_FILE, 'a', encoding='utf-8').close()
    if not os.path.exists(POINTS_FILE):
        with open(POINTS_FILE, 'w', encoding='utf-8') as f:
            json.dump({
                "points": [], 
                "rtl": {"mode": "rtl", "alt": 20, "lat": 0, "lon": 0}
            }, f)
    
    # بدء مؤشر تحديث موقع الدرون إذا كان اتصال الدرون مفعلاً
    if DRONE_CONNECTIVITY_ENABLED:
        loc_thread = threading.Thread(target=update_drone_location, daemon=True)
        loc_thread.start()
        log_event("بدء مؤشر تحديث موقع الدرون", "معلومات")
    
    print("*"*50)
    print(f"بدء الخادم على http://0.0.0.0:{SERVER_PORT}")
    print("*"*50)
    app.run(host='0.0.0.0', port=SERVER_PORT, debug=False, threaded=True)
