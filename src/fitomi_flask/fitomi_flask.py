import rclpy
from rclpy.node import Node
from custom_interfaces.srv import LogMessage, ImgLoad
from cv_bridge import CvBridge
from flask import Flask, render_template, Response, request, redirect, url_for, session, jsonify
from sensor_msgs.msg import Image
import threading
import sqlite3
import numpy as np
import cv2
import os
import time
import base64

app = Flask(__name__)
app.secret_key = 'fitomi_secret'

DATABASE = 'users.db'
bridge = CvBridge()

frame_lock = threading.Lock()
latest_frame = None

class FlaskRosNode(Node):
    def __init__(self):
        super().__init__('flask_ros_node')
        self.bridge = CvBridge()
        self.logs = []
        self.sub = self.create_subscription(Image, '/send_img', self.image_callback, 10)
        self.create_service(LogMessage, 'log_message', self.handle_log_service)
        self.create_service(ImgLoad, 'img_load', self.handle_imgload)
        self.uploaded_image_path = "static/uploaded.jpg"

    def image_callback(self, msg):
        global latest_frame
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, jpeg = cv2.imencode('.jpg', cv_img)
            with frame_lock:
                latest_frame = jpeg.tobytes()
        except Exception as e:
            self.get_logger().error(f"send_img callback error: {e}")

    def handle_log_service(self, request, response):
        log_entry = f"[LOG] {request.log}"
        self.logs.append(log_entry)
        self.get_logger().info(log_entry)
        response.answer = True
        response.message = "Log received"
        return response

    def handle_imgload(self, request, response):
        try:
            cv_img = cv2.imread(self.uploaded_image_path)
            if cv_img is None:
                self.get_logger().error("[IMG] ImgLoad: 업로드 이미지 없음")
                return response
            response.img = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            self.get_logger().info("[IMG] ImgLoad 응답 성공")
        except Exception as e:
            self.get_logger().error(f"[IMG] ImgLoad 처리 중 오류: {e}")
        return response

# DB 초기화
def init_db():
    if not os.path.exists(DATABASE):
        conn = sqlite3.connect(DATABASE)
        c = conn.cursor()
        c.execute("CREATE TABLE IF NOT EXISTS users (id TEXT PRIMARY KEY, password TEXT)")
        conn.commit()
        conn.close()

@app.route('/', methods=['GET', 'POST'])
def login():
    conn = sqlite3.connect(DATABASE)
    c = conn.cursor()
    c.execute("SELECT id FROM users")
    user_list = [row[0] for row in c.fetchall()]
    conn.close()

    if request.method == 'POST':
        user_id = request.form['id']
        password = request.form['password']

        conn = sqlite3.connect(DATABASE)
        c = conn.cursor()
        c.execute("SELECT * FROM users WHERE id = ?", (user_id,))
        user = c.fetchone()

        if user and user[1] == password:
            session['user_id'] = user_id
            return redirect(url_for('dashboard'))
        else:
            return render_template('my_confirm_register.html', id=user_id, password=password)

    return render_template('my_login.html', user_list=user_list)

@app.route('/register', methods=['POST'])
def register():
    data = request.get_json()
    user_id = data['id']
    password = data['password']
    conn = sqlite3.connect(DATABASE)
    c = conn.cursor()
    c.execute("INSERT INTO users (id, password) VALUES (?, ?)", (user_id, password))
    conn.commit()
    conn.close()
    session['user_id'] = user_id
    return redirect(url_for('dashboard'))

@app.route('/dashboard')
def dashboard():
    if 'user_id' not in session:
        # return redirect(url_for('login'))
        # 세션 만료 시 로그 자동 저장
        if ros_node.logs:
            conn = sqlite3.connect(DATABASE)
            c = conn.cursor()
            c.execute("CREATE TABLE IF NOT EXISTS logs (timestamp TEXT, message TEXT)")
            for line in ros_node.logs:
                c.execute("INSERT INTO logs VALUES (datetime('now'), ?)", (line,))
            conn.commit()
            conn.close()
            ros_node.logs.clear()  # 로그 초기화
        return redirect(url_for('login'))
    return render_template('dashboard.html', user_id=session['user_id'])

@app.route('/video_feed')
def video_feed():
    def generate():
        print("[VIDEO] video_feed stream activated")
        while True:
            with frame_lock:
                frame = latest_frame
            if frame:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.1)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/upload_image', methods=['POST'])
def upload_image():
    if 'image' not in request.files:
        return jsonify({"error": "No image uploaded"}), 400
    file = request.files['image']
    filepath = os.path.join("static", "uploaded.jpg")
    file.save(filepath)
    ros_node.uploaded_image_path = filepath
    return jsonify({"status": "sent"})

@app.route('/get_recommended_image')
def get_recommended_image():
    fallback = cv2.imread("static/unavailable.jpg")
    _, buffer = cv2.imencode('.jpg', fallback)
    b64_img = base64.b64encode(buffer).decode('utf-8')
    return jsonify({"img": b64_img})

@app.route('/get_logs')
def get_logs():
    global ros_node
    return jsonify({"logs": ros_node.logs[-50:]})

@app.route('/save_logs', methods=['POST'])
def save_logs():
    global ros_node
    conn = sqlite3.connect(DATABASE)
    c = conn.cursor()
    c.execute("CREATE TABLE IF NOT EXISTS logs (timestamp TEXT, message TEXT)")
    for line in ros_node.logs:
        c.execute("INSERT INTO logs VALUES (datetime('now'), ?)", (line,))
    conn.commit()
    conn.close()
    return jsonify({"status": "success"})

@app.route('/logout')
def logout():
    global ros_node
    if ros_node.logs:
        conn = sqlite3.connect(DATABASE)
        c = conn.cursor()
        c.execute("CREATE TABLE IF NOT EXISTS logs (timestamp TEXT, message TEXT)")
        for line in ros_node.logs:
            c.execute("INSERT INTO logs VALUES (datetime('now'), ?)", (line,))
        conn.commit()
        conn.close()

        # 로그 초기화
        ros_node.logs.clear()

    # 세션 종료
    session.pop('user_id', None)
    return redirect(url_for('login'))

@app.route('/clear_logs_on_exit', methods=['POST'])
def clear_logs_on_exit():
    global ros_node
    print("[INFO] 브라우저 종료 감지 → 로그 초기화")
    ros_node.logs.clear()
    return '', 204


if __name__ == '__main__':
    init_db()
    rclpy.init()
    ros_node = FlaskRosNode()
    threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True).start()
    app.run(host='0.0.0.0', port=5000)
