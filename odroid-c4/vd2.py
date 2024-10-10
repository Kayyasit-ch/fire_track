import socket
import cv2
import numpy as np
import time  

# ตั้งค่า IP หรือ mDNS hostname ของ ESP32-CAM
host = 'esp32cam.local'  # ถ้า mDNS ไม่ทำงาน ให้ใช้ IP แทน เช่น '192.168.x.x'
port = 8080

# ตั้งค่า socket สำหรับการเชื่อมต่อ
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((host, port))

# โหลด Haar Cascade สำหรับการตรวจจับร่างกายเต็มตัว
fullbody_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')

# สร้าง buffer เพื่อเก็บข้อมูลภาพ
data = b""  
use_thermal_infrared_filter = False  # ตัวแปรเพื่อสลับการใช้ฟิลเตอร์ความร้อน

def apply_thermal_infrared_filter(image):
    """ใช้ฟิลเตอร์ความร้อนกับภาพ."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    colored_map = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
    combined = cv2.addWeighted(image, 0.5, colored_map, 0.5, 0)
    return combined

def analyze_situation(bodies):
    """วิเคราะห์สถานการณ์จากการตรวจจับร่างกาย."""
    if len(bodies) > 5:
        return "Critical: Multiple victims detected", len(bodies)
    elif len(bodies) > 0:
        return "Normal: 1 or more victims detected", len(bodies)
    else:
        return "Safe: No victims detected", 0

try:
    while True:
        # รับข้อมูลจาก ESP32-CAM
        packet = client_socket.recv(4096)
        if not packet:
            break
        data += packet

        # ตรวจหาช่วงที่เป็นข้อมูลของ JPEG ใน data
        a = data.find(b'\xff\xd8')  # เริ่มต้นของ JPEG
        b = data.find(b'\xff\xd9')  # สิ้นสุดของ JPEG

        if a != -1 and b != -1:
            jpg = data[a:b+2]
            data = data[b+2:]

            # แปลงข้อมูล JPEG เป็นภาพ
            img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

            if img is not None:
                img_resized = cv2.resize(img, (420, 480))
                gray = cv2.cvtColor(img_resized, cv2.COLOR_BGR2GRAY)

                # ตรวจจับร่างกายในภาพ grayscale
                bodies = fullbody_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=3)

                for (x, y, w, h) in bodies:
                    cv2.rectangle(img_resized, (x, y), (x+w, y+h), (0, 255, 0), 2)

                # วิเคราะห์สถานการณ์
                situation_message, num_people = analyze_situation(bodies)

                # สลับการใช้ฟิลเตอร์ความร้อนเมื่อกด 'c'
                if cv2.waitKey(1) & 0xFF == ord('c'):
                    use_thermal_infrared_filter = not use_thermal_infrared_filter

                # ใช้ฟิลเตอร์ความร้อนหากเปิดใช้งาน
                if use_thermal_infrared_filter:
                    img_resized = apply_thermal_infrared_filter(img_resized)

                # แสดงข้อความวิเคราะห์สถานการณ์บนภาพ
                cv2.putText(img_resized, situation_message, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # แสดงภาพที่ผ่านการประมวลผล
                cv2.imshow('ESP32-CAM', img_resized)

            # ใช้ time.sleep เพื่อให้รับข้อมูลทุกๆ 0.01 วินาที
            time.sleep(0.01)  # Delay for 0.01 seconds before next loop

            if cv2.waitKey(1) & 0xFF == 27:  # Exit when 'ESC' is pressed
                break
finally:
    client_socket.close()
    cv2.destroyAllWindows()
