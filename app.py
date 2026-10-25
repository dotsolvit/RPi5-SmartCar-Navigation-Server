import json
import math
import os
import threading
from flask import Flask, Response, jsonify, render_template, request
import paho.mqtt.client as mqtt

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

app = Flask(__name__)

MAP_SIZE = 80  
raw_map = [[0 for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]

# Позиція в метрах, курс у градусах, заряд у %
robot_pose = {"x": 4.0, "y": 4.0, "theta_deg": 0.0, "battery_pct": 100} 
current_path_m = [] # Дрібний шлях (для UI)
route_status = "Очікування цілі"
robot_status = "stopped"  
navigation_mode = "manual"  
goal_pos_m = None  

ROBOT_SAFETY_RADIUS = 2 
MAP_FILE_PATH = "map_data.txt"

change_event = threading.Event()

def get_inflated_matrix():
    matrix = [[1 for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            if raw_map[y][x] == 1:
                for ny in range(max(0, y - ROBOT_SAFETY_RADIUS), min(MAP_SIZE, y + ROBOT_SAFETY_RADIUS + 1)):
                    for nx in range(max(0, x - ROBOT_SAFETY_RADIUS), min(MAP_SIZE, x + ROBOT_SAFETY_RADIUS + 1)):
                        matrix[ny][nx] = 0
    return matrix

def vectorize_path(full_path):
    """Залишає в маршруті тільки точки стаpoints, зламу (поворотів) та фінішу"""
    if len(full_path) <= 2:
        return full_path
        
    vector_path = [full_path[0]] # Додаємо точку старту
    
    for i in range(1, len(full_path) - 1):
        prev_pt = full_path[i-1]
        curr_pt = full_path[i]
        next_pt = full_path[i+1]
        
        # Рахуємо вектори напрямку
        dx1 = curr_pt["x"] - prev_pt["x"]
        dy1 = curr_pt["y"] - prev_pt["y"]
        dx2 = next_pt["x"] - curr_pt["x"]
        dy2 = next_pt["y"] - curr_pt["y"]
        
        # Перевірка на колінеарність (чи лежать на одній прямій лінії)
        # Використовуємо косоугольний добуток векторів з невеликим допуском на округлення
        cross_product = dx1 * dy2 - dy1 * dx2
        if abs(cross_product) > 0.001:
            # Напрямок змінився — це точка повороту!
            vector_path.append(curr_pt)
            
    vector_path.append(full_path[-1]) # Додаємо фінішну точку
    return vector_path

def calculate_astar_path(start_x_m, start_y_m, end_x_m, end_y_m):
    global route_status, current_path_m
    
    start_x = int(start_x_m * 10)
    start_y = int(start_y_m * 10)
    end_x = int(end_x_m * 10)
    end_y = int(end_y_m * 10)
    
    search_matrix = get_inflated_matrix()
    grid = Grid(matrix=search_matrix)
    
    try:
        start_node = grid.node(start_x, start_y)
        end_node = grid.node(end_x, end_y)
        
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start_node, end_node, grid)
        
        if path and len(path) > 1:
            # 1. Створюємо повний дрібний маршрут для Canvas
            current_path_m = [{"x": round(n.x / 10.0 + 0.05, 2), "y": round(n.y / 10.0 + 0.05, 2)} for n in path]
            route_status = f"Маршрут побудовано (точок: {len(current_path_m)})"
            
            # 2. Укрупнюємо шлях для реальної машинки (тільки прямі та повороти)
            macro_path = vectorize_path(current_path_m)
            print(f"Сервер: Шлях укрупнено! З {len(current_path_m)} точок до {len(macro_path)} ключових векторів.")
            
            # Шлемо машинці ТІЛЬКИ укрупнені ключові точки
            mqtt_client.publish("robot/path", json.dumps({"points": macro_path}), qos=1)
            return True
        else:
            current_path_m = []
            route_status = "Маршрут побудувати неможливо (заблоковано)"
            return False
    except Exception as e:
        current_path_m = []
        route_status = f"Помилка навігації: {str(e)}"
        return False

# --- MQTT Обробка ---
def on_message(client, userdata, msg):
    global robot_pose, robot_status, raw_map, route_status, goal_pos_m, navigation_mode
    try:
        payload = json.loads(msg.payload.decode())
        
        if msg.topic == "robot/telemetry":
            robot_pose["x"] = float(payload["x"])
            robot_pose["y"] = float(payload["y"])
            robot_pose["theta_deg"] = float(payload["theta_deg"])
            # Зчитуємо заряд акумулятора, якщо його немає в пакеті — ставимо 100 за замовчуванням
            robot_pose["battery_pct"] = int(payload.get("battery_pct", 100))
            if "status" in payload:
                robot_status = payload["status"]
            change_event.set()

            
        elif msg.topic == "robot/obstacle_scan":
            print("Сервер: Отримано повне сканування перешкод 180° в мм.")
            angles = payload.get("angles", [])       
            distances_mm = payload.get("distances", []) 
            
            rx, ry = robot_pose["x"], robot_pose["y"]
            r_theta = robot_pose["theta_deg"]
            
            servo_x = rx + 0.06 * math.sin(math.radians(r_theta))
            servo_y = ry + 0.06 * math.cos(math.radians(r_theta))
            
            walls_found = 0
            for angle_rel, dist_mm in zip(angles, distances_mm):
                dist_cm = dist_mm / 10.0
                if 20 <= dist_cm <= 100:
                    dist_m = dist_mm / 1000.0
                    total_angle_deg = (r_theta + angle_rel) % 360.0
                    wall_x = servo_x + dist_m * math.sin(math.radians(total_angle_deg))
                    wall_y = servo_y + dist_m * math.cos(math.radians(total_angle_deg))
                    cx = int(wall_x * 10)
                    cy = int(wall_y * 10)
                    if 0 <= cx < MAP_SIZE and 0 <= cy < MAP_SIZE:
                        raw_map[cy][cx] = 1 
                        walls_found += 1
                        
            print(f"Карту сервера оновлено. Додано клітинок стін: {walls_found}")
            
            if navigation_mode == "auto" and goal_pos_m:
                route_status = "Автомат: Перерахунок в обхід перешкоди..."
                success = calculate_astar_path(robot_pose["x"], robot_pose["y"], goal_pos_m["x"], goal_pos_m["y"])
                if success:
                    mqtt_client.publish("robot/control", json.dumps({"command": "start"}), qos=1)
            else:
                route_status = "Перешкоду знайдено! Чекаю повторного Старту."
            change_event.set()
    except Exception as e:
        print(f"Помилка сервера при обробці MQTT: {e}")

mqtt_client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.on_message = on_message

def start_mqtt():
    try:
        mqtt_client.connect("localhost", 1883, 60)
        mqtt_client.subscribe([("robot/telemetry", 1), ("robot/obstacle_scan", 1)])
        mqtt_client.loop_forever()
    except Exception as e:
        print(f"Помилка Mosquitto: {e}")

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/control", methods=["POST"])
def control_robot():
    global robot_status, current_path_m, route_status, navigation_mode, raw_map
    data = request.get_json()
    action = data.get("action")
    
    if action == "save_map":
        try:
            with open(MAP_FILE_PATH, "w") as f:
                for y in range(MAP_SIZE):
                    for x in range(MAP_SIZE):
                        if raw_map[y][x] == 1: f.write(f"{x},{y}\n")
            route_status = "Карту перешкод збережено."
        except Exception as e: route_status = f"Помилка: {str(e)}"
        change_event.set()
        return jsonify({"status": "success"})
        
    elif action == "load_map":
        if not os.path.exists(MAP_FILE_PATH):
            route_status = "Файл карти не знайдено!"
        else:
            try:
                raw_map = [[0 for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]
                with open(MAP_FILE_PATH, "r") as f:
                    for line in f:
                        if line.strip():
                            cx, cy = map(int, line.strip().split(","))
                            raw_map[cy][cx] = 1
                route_status = "Карту перешкод завантажено."
                if goal_pos_m:
                    calculate_astar_path(robot_pose["x"], robot_pose["y"], goal_pos_m["x"], goal_pos_m["y"])
            except Exception as e: route_status = f"Помилка: {str(e)}"
        change_event.set()
        return jsonify({"status": "success"})
        
    elif action == "clear_map":
        raw_map = [[0 for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]
        current_path_m = []
        route_status = "Карту повністю очищено."
        change_event.set()
        return jsonify({"status": "success"})

    if action in ["manual", "auto"]:
        navigation_mode = action
        change_event.set()
        return jsonify({"status": "success"})
        
    if action == "start":
        if not current_path_m:
            route_status = "Помилка: Спершу вкажіть ціль!"
            change_event.set()
            return jsonify({"status": "ignored"})

    mqtt_client.publish("robot/control", json.dumps({"command": action}), qos=1)
    return jsonify({"status": "success"})

@app.route("/set_goal", methods=["POST"])
def set_goal():
    global current_path_m, robot_pose, route_status, robot_status, goal_pos_m
    data = request.get_json()
    cell_x, cell_y = int(data["x"]), int(data["y"])
    goal_pos_m = {"x": round(cell_x / 10.0 + 0.05, 2), "y": round(cell_y / 10.0 + 0.05, 2)}
    
    if robot_status == "arrived":
        robot_status = "stopped"
        
    calculate_astar_path(robot_pose["x"], robot_pose["y"], goal_pos_m["x"], goal_pos_m["y"])
    change_event.set()
    return jsonify({"status": "success"})


# Потік трансляції комбінованих даних у браузер (Server-Sent Events)
@app.route("/stream")
def stream():
    def event_stream():
        while True:
            # Формуємо єдиний пакет даних для фронтенду
            packet = {
                "robot": robot_pose,
                "robot_status": robot_status,
                "path": current_path_m,
                "route_status": route_status,
                "nav_mode": navigation_mode,
                "raw_map": raw_map,  
                "safety_radius": ROBOT_SAFETY_RADIUS
            }
            yield f"data: {json.dumps(packet)}\n\n"
            # Чекаємо наступної зміни позиції або оновлення карти (блокуюче очікування)
            change_event.wait(timeout=0.3)
            change_event.clear()

    # Повертаємо потік з правильним MIME-типом text/event-stream
    return Response(event_stream(), mimetype="text/event-stream")


if __name__ == "__main__":
    mqtt_thread = threading.Thread(target=start_mqtt, daemon=True)
    mqtt_thread.start()
    app.run(host="0.0.0.0", port=5000, debug=False)