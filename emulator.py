import json
import math
import os
import time
import paho.mqtt.client as mqtt

# Налаштування підключення до брокера Mosquitto на RPi5
MQTT_BROKER = "localhost"
MQTT_PORT = 1883

# Фізичні константи вашої кімнатної машинки
MAX_SPEED_MPS = 0.65       # Лінійна швидкість: 65 см/сек = 0.65 м/с
MAX_TURN_SPEED_DPS = 180.0  # Кутова швидкість: 180 град/сек

# Крок фізики таймера = 0.1 сек (100 мс -> 10 Гц телеметрії)
DT = 0.1
TURN_PER_STEP = MAX_TURN_SPEED_DPS * DT  # Максимальний оберт за 1 крок = 18 градусів

# Глобальний стан віртуального робота (починає з центру карти в метрах)
robot_x = 4.0        
robot_y = 4.0        
robot_theta_deg = 0.0
is_running = False   
target_path_m = []     # Укрупнений векторний маршрут
robot_battery = 81  # Задаємо якийсь стартовий рівень заряду (81%)
last_status = "stopped"
telemetry_dirty = True # Прапорець для відправки фінального пакета

# Прості глобальні лічильники кроків таймера
cal_counter = 0
battery_counter = 0

# Карта прихованих перешкод для HIL-тестування
secret_obstacles = set()
SECRET_MAP_PATH = "secret_map.txt"

# Змінні для коливання голови далекоміра Sharp під час руху (коридор безпеки +/- 11°)
moving_scan_angle = -11.0
moving_scan_dir = 1.0

def load_secret_obstacles():
    """Зчитує координати прихованих стін, які машинка має виявити далекоміром"""
    global secret_obstacles
    secret_obstacles.clear()
    if os.path.exists(SECRET_MAP_PATH):
        with open(SECRET_MAP_PATH, "r") as f:
            for line in f:
                if line.strip():
                    cx, cy = map(int, line.strip().split(","))
                    secret_obstacles.add((cx, cy))
        print(f"Емулятор: Завантажено приховані перешкоди ({len(secret_obstacles)} точок)")
    else:
        print("Емулятор: Файл secret_map.txt не знайдено. Карта порожня.")

def normalize_heading(angle):
    """Приводить кут до стандартного навігаційного діапазону [0.0, 360.0)"""
    while angle >= 360.0: angle -= 360.0
    while angle < 0.0: angle += 360.0
    return angle

def on_connect(client, userdata, flags, rc, properties=None):
    print("Емулятор підключився до Mosquitto!")
    client.subscribe([("robot/control", 1), ("robot/path", 1)])

def on_message(client, userdata, msg):
    global is_running, target_path_m, telemetry_dirty, last_status
    try:
        payload = json.loads(msg.payload.decode())
        
        if msg.topic == "robot/control":
            cmd = payload.get("command")
            if cmd == "start" and len(target_path_m) > 0:
                is_running = True
                telemetry_dirty = True
                last_status = "moving"
                print("Емулятор: РУХ ДОЗВОЛЕНО (СТАРТ)")
            elif cmd == "stop":
                is_running = False
                telemetry_dirty = True
                last_status = "stopped"
                print("Емулятор: РУХ ПРИМУСОВО ЗУПИНЕНО")
            elif cmd == "calibrate":
                is_running = False
                telemetry_dirty = True
                last_status = "calibrating"
                print("Емулятор: Запущено калібрування компасу...")
                
        elif msg.topic == "robot/path":
            points = payload.get("points", [])
            target_path_m = [(pt["x"], pt["y"]) for pt in points]
            
            # Читання першої точки
            if len(target_path_m) > 0:
                first_pt = target_path_m[0]  # Беремо перший кортеж (x, y) зі списку
                dx = first_pt[0] - robot_x   # first_pt[0] — це координата X
                dy = first_pt[1] - robot_y   # first_pt[1] — це координата Y
                if math.sqrt(dx**2 + dy**2) < 0.08:
                    target_path_m.pop(0)
                    
            telemetry_dirty = True
            if last_status in ["arrived", "scanning"]:
                last_status = "stopped"
            print(f"Емулятор: Отримано новий укрупнений шлях ({len(target_path_m)} точок)")
            
    except Exception as e:
        print(f"Помилка емулятора при обробці повідомлення: {e}")

def check_corridor_obstacles(max_scan_dist_m):
    """Емулює динамічний вузький далекомір (+/- 11°) з обмеженням по дистанції руху"""
    global robot_x, robot_y, robot_theta_deg, secret_obstacles, moving_scan_angle, moving_scan_dir
    
    moving_scan_angle += moving_scan_dir * 4.0
    if abs(moving_scan_angle) >= 11.0:
        moving_scan_dir *= -1.0
        
    total_angle = (robot_theta_deg + moving_scan_angle) % 360.0
    servo_x = robot_x + 0.06 * math.sin(math.radians(robot_theta_deg))
    servo_y = robot_y + 0.06 * math.cos(math.radians(robot_theta_deg))
    
    limit_dist_m = min(1.0, max_scan_dist_m)
    steps = int(limit_dist_m / 0.02)
    for step in range(10, steps + 1):
        dist_m = step * 0.02
        check_x = servo_x + dist_m * math.sin(math.radians(total_angle))
        check_y = servo_y + dist_m * math.cos(math.radians(total_angle))
        cx = int(check_x * 10)
        cy = int(check_y * 10)
        
        if (cx, cy) in secret_obstacles:
            return int(dist_m * 1000)
            
    return None

def execute_180_degree_scan(client):
    """Повне широке сканування 180° під час аварійної зупинки перед стіною"""
    global robot_x, robot_y, robot_theta_deg, secret_obstacles
    print("Емулятор: СТОП перед перешкодою. Запуск панорамного огляду...")
    
    servo_x = robot_x + 0.06 * math.sin(math.radians(robot_theta_deg))
    servo_y = robot_y + 0.06 * math.cos(math.radians(robot_theta_deg))
    
    scan_angles = list(range(-90, 91, 5))
    scan_distances_mm = []
    
    for sa in scan_angles:
        total_angle = (robot_theta_deg + sa) % 360.0
        found_dist_mm = 1500
        
        for dist_mm in range(200, 1501, 20):
            dist_m = dist_mm / 1000.0
            cx = int((servo_x + dist_m * math.sin(math.radians(total_angle))) * 10)
            cy = int((servo_y + dist_m * math.cos(math.radians(total_angle))) * 10)
            if (cx, cy) in secret_obstacles:
                found_dist_mm = dist_mm
                break
                
        scan_distances_mm.append(found_dist_mm)
        time.sleep(0.02)
        
    scan_payload = {"angles": scan_angles, "distances": scan_distances_mm}
    client.publish("robot/obstacle_scan", json.dumps(scan_payload), qos=1)
    print("Емулятор: Результати сканування перешкоди 180° передано в MQTT.")

def run_emulator():
    global robot_x, robot_y, robot_theta_deg, is_running, target_path_m, robot_battery, last_status, telemetry_dirty
    
    load_secret_obstacles()
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    
    try: 
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e: 
        print(f"Помилка підключення до MQTT: {e}")
        return
        
    client.loop_start()
    print("Емулятор фізики руху машинки успішно запущено.")

    try:
        while True:
            has_moved = False
            
            # --- РЕЖИМ 1: Калібрування компасу на місці ---
            if last_status == "calibrating":
                global cal_counter
                robot_theta_deg = (robot_theta_deg + 15.0) % 360.0
                has_moved = True
                telemetry_dirty = True
                
                # Класичний інтуїтивний лічильник
                cal_counter += 1
                if cal_counter >= 20:
                    cal_counter = 0
                    last_status = "stopped"
                    print("Емулятор: Калібрування магнітометра завершено.")
            
            # --- РЕЖИМ 2: Рух за вектором шляху ---
            elif is_running and len(target_path_m) > 0:
                # Читання першого елемента списку 
                current_node = target_path_m[0]
                target_x_m = current_node[0]
                target_y_m = current_node[1]
                
                dx = target_x_m - robot_x
                dy = target_y_m - robot_y
                distance_to_node = math.sqrt(dx**2 + dy**2)
                
                # Перевірка досягнення поточної точки
                if distance_to_node < 0.08: 
                    target_path_m.pop(0)
                    if len(target_path_m) == 0:
                        is_running = False
                        last_status = "arrived"
                        telemetry_dirty = True
                        print("Емулятор: Фінальна ціль успішно досягнута!")
                        continue
                    else:
                        print("Емулятор: Перехід до наступного вектора.")
                        continue
                
                target_angle_deg = normalize_heading(math.degrees(math.atan2(dx, dy)))
                
                angle_diff = target_angle_deg - robot_theta_deg
                if angle_diff > 180.0: angle_diff -= 360.0
                if angle_diff < -180.0: angle_diff += 360.0
                
                # ЕТАП А: Обертання корпусу на місці
                if abs(angle_diff) > 5.0: 
                    turn_step = math.copysign(TURN_PER_STEP, angle_diff)
                    if abs(turn_step) > abs(angle_diff):
                        robot_theta_deg = target_angle_deg
                    else:
                        robot_theta_deg += turn_step
                    robot_theta_deg = normalize_heading(robot_theta_deg)
                    has_moved = True 
                
                # ЕТАП Б: Поступальний рух вперед за вектором
                else:
                    obstacle_dist_mm = check_corridor_obstacles(distance_to_node)
                    
                    if obstacle_dist_mm is not None:
                        is_running = False
                        last_status = "scanning"
                        
                        client.publish("robot/telemetry", json.dumps({
                            "x": round(robot_x, 2), "y": round(robot_y, 2),
                            "theta_deg": round(robot_theta_deg, 1), "status": "scanning"
                        }))
                        
                        execute_180_degree_scan(client)
                        last_status = "stopped"
                        telemetry_dirty = True
                        continue
                    
                    robot_x += MAX_SPEED_MPS * DT * math.sin(math.radians(robot_theta_deg))
                    robot_y += MAX_SPEED_MPS * DT * math.cos(math.radians(robot_theta_deg))
                    has_moved = True

                if is_running: 
                    last_status = "moving"
                    
            # Логіка плавного мікро-розряду тільки під час руху машинки
            if is_running and has_moved:
                # Зменшуємо заряд дуже повільно (приблизно на 1% за 300 кроків руху)
                global battery_counter
                battery_counter += 1
                if battery_counter >= 300:
                    battery_counter = 0
                    if robot_battery > 0:
                        robot_battery -= 1

            # Оптимізація трафіку
            if has_moved or telemetry_dirty:
                telemetry = {
                    "x": round(robot_x, 2), 
                    "y": round(robot_y, 2),
                    "theta_deg": round(robot_theta_deg, 1), 
                    "battery_pct": robot_battery,  # Додаємо рівень заряду в MQTT пакет
                    "status": last_status
                }
                client.publish("robot/telemetry", json.dumps(telemetry))
                if not has_moved: 
                    telemetry_dirty = False
            
            time.sleep(DT)

    except KeyboardInterrupt: 
        print("\nЕмулятор успішно вимкнено користувачем.")
    finally: 
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    run_emulator()
