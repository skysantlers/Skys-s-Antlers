#!/usr/bin/env python
"""
Ground Station Uygulaması – Kalabalık Yönetimi & Trafik Analizi (Tam Görev)
Görev İçeriği:
- Yukarıdan çekilen görüntü üzerinden kalabalık insan topluluklarını (ör. festival alanı, toplu etkinlik)
  gözlemleyip, yoğunluk haritası çıkarmak ve acil durum senaryoları için erken uyarı sistemi oluşturmak.
- Trafik akışını analiz ederek, tıkanan bölgeleri tespit etmek ve trafik ışığı optimizasyonu önerileri sunmak.
Bu kod:
  - DroneKit ile Mission Planner SITL üzerinden telemetri verilerini alır.
  - OpenCV ile gerçek zamanlı video akışı üzerinden insan ve araç tespiti yapar.
  - Tespit sonuçlarına göre kalabalık yoğunluğu haritası ve trafik analizi yapar.
  - Tkinter tabanlı GUI ile telemetri, video ve analiz sonuçlarını gerçek zamanlı gösterir.
  - Otonom uçuş ve kontrollü uçuş yetenekleri eklenmiştir.
Çalıştırma:
    Terminal veya Komut İstemi'nde:
    python ground_station_missionplanner.py
Gereksinimler: dronekit, opencv-python, pillow, tensorflow (veya pytorch)
"""

import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import cv2
import threading
import time
import random
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
import queue

# Kuyruklar: Telemetri ve işlenmiş video karelerini aktaracak
telemetry_queue = queue.Queue()
frame_queue = queue.Queue()

#######################################
# 1. TELEMETRİ VERİLERİNİN OKUNMASI   #
#######################################


def telemetry_thread(vehicle):
    """
    Mission Planner SITL üzerinden telemetri verilerini (GPS, irtifa, pil, uçuş modu) okur.
    Her saniye güncellenen veriler telemetry_queue'ya aktarılır. Bağlantı yoksa simülasyon örneği kullanılır.
    """
    while True:
        try:
            if vehicle:
                gps = vehicle.location.global_frame
                altitude = vehicle.location.global_relative_frame.alt
                battery = (
                    vehicle.battery.level if vehicle.battery.level is not None else 0
                )
                mode = vehicle.mode.name
            else:
                # Simülasyon örnek verileri:
                gps = type("GPS", (object,), {"lat": 41.015137, "lon": 28.979530})()
                altitude = 10.0
                battery = 95
                mode = "GUIDED"
            telemetry_data = {
                "gps": gps,
                "altitude": altitude,
                "battery": battery,
                "mode": mode,
            }
            telemetry_queue.put(telemetry_data)
        except Exception as e:
            print("Telemetri okuma hatası:", e)
        time.sleep(1)


#######################################
# 2. VIDEO AKIŞI VE GÖRÜNTÜ İŞLEME    #
#######################################

# İnsan tespiti için HOGDescriptor veya derin öğrenme modeli yükle
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# Araç tespiti için YOLO modeli yükle (örneğin, yolov3.weights ve yolov3.cfg dosyaları)
net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers().flatten()]


def detect_people(frame):
    """OpenCV HOGDescriptor ile insan tespiti yapar."""
    rects, _ = hog.detectMultiScale(frame, winStride=(4, 4), padding=(8, 8), scale=1.05)
    return rects


def detect_vehicles(frame):
    """YOLO ile araç tespiti yapar."""
    height, width, channels = frame.shape
    blob = cv2.dnn.blobFromImage(
        frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False
    )
    net.setInput(blob)
    outs = net.forward(output_layers)

    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5 and class_id == 2:  # 2: araç sınıfı
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
    return boxes


def video_thread():
    """
    Kameradan alınan video akışı üzerinden gerçek zamanlı insan ve araç tespiti yapar.
    - İnsan tespiti için HOGDescriptor kullanılır.
    - Araç tespiti için YOLO modeli kullanılır.
    - Tespitlerin yoğunluğuna göre, yoğunluk haritası (heatmap) üretilir.
    - Eşik değerlerin üzerinde kalabalık veya trafik tespit edilirse erken uyarı ve trafik ışığı optimizasyonu önerileri overlay olarak eklenir.
    İşlenmiş kare, frame_queue'ya aktarılır.
    """
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Video kaynağı açılamadı!")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # İnsan tespiti
        people_boxes = detect_people(frame)
        num_people = len(people_boxes)

        # Araç tespiti
        vehicle_boxes = detect_vehicles(frame)
        num_vehicles = len(vehicle_boxes)

        # Kalabalık yoğunluğu için bir ısı haritası oluşturma
        h, w, _ = frame.shape
        density_map = np.zeros((h, w), dtype=np.uint8)
        for x, y, w_box, h_box in people_boxes:
            cv2.circle(density_map, (x + w_box // 2, y + h_box // 2), 20, 255, -1)

        # Yoğunluk haritasını bulanıklaştırarak yumuşatın
        density_map = cv2.GaussianBlur(density_map, (51, 51), 0)

        # Renk haritası uygula (örn. JET renk paleti)
        heatmap = cv2.applyColorMap(density_map, cv2.COLORMAP_JET)

        # Isı haritasını orijinal görüntü üzerine alfa karışımı ile ekleyelim
        alpha_heat = 0.4
        frame = cv2.addWeighted(heatmap, alpha_heat, frame, 1 - alpha_heat, 0)

        # İnsan ve araç kutularını çizelim
        for x, y, w_box, h_box in people_boxes:
            cv2.rectangle(frame, (x, y), (x + w_box, y + h_box), (0, 255, 0), 2)

        for x, y, w_box, h_box in vehicle_boxes:
            cv2.rectangle(frame, (x, y), (x + w_box, y + h_box), (255, 0, 0), 2)

        # Erken uyarı ve trafik ışığı optimizasyonu kontrolleri
        if num_people > 30:
            cv2.putText(
                frame,
                "Erken Uyarı: Yoğun Kalabalik!",
                (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                3,
            )
        else:
            cv2.putText(
                frame,
                "Kalabalik: Normal",
                (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

        if num_vehicles > 10:
            cv2.putText(
                frame,
                "Trafik Sikinligi: Işık Optimizasyonu ONERILIYOR!",
                (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
            )
        else:
            cv2.putText(
                frame,
                "Trafik: Normal",
                (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

        # Üzerine, tespit sayılarını da yazdıralım
        cv2.putText(
            frame,
            f"People Count: {num_people}",
            (10, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            f"Vehicle Count: {num_vehicles}",
            (10, 160),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )

        # İşlenmiş kareyi kuyruga aktaralım
        frame_queue.put(frame)
        time.sleep(0.03)  # Yaklaşık 30 FPS

    cap.release()


#######################################
# 3. YER İSTASYONU GUI TASARIMI       #
#######################################


class GroundStationGUI:
    def __init__(self, root, vehicle):
        self.root = root
        self.vehicle = vehicle
        self.root.title("İHA Yer İstasyonu (Mission Planner SITL & Görev Analizi)")
        # Telemetri bilgilerini gösterecek etiketler
        self.mode_label = ttk.Label(root, text="Mod: N/A", font=("Helvetica", 16))
        self.mode_label.pack(pady=5)
        self.altitude_label = ttk.Label(
            root, text="İrtifa: N/A", font=("Helvetica", 16)
        )
        self.altitude_label.pack(pady=5)
        self.gps_label = ttk.Label(root, text="GPS: N/A", font=("Helvetica", 16))
        self.gps_label.pack(pady=5)
        self.battery_label = ttk.Label(root, text="Pil: N/A", font=("Helvetica", 16))
        self.battery_label.pack(pady=5)
        # Video akışı ve analiz sonuçlarını gösterecek panel
        self.video_panel = tk.Label(root)
        self.video_panel.pack(padx=10, pady=10)
        # GUI güncelleme döngüsünü başlat
        self.update_gui()

    def update_gui(self):
        """
        Kuyruklardan telemetri ve video verilerini çekerek GUI üzerindeki bilgileri günceller.
        """
        try:
            telemetry_data = telemetry_queue.get_nowait()
            self.mode_label.config(text=f"Mod: {telemetry_data['mode']}")
            self.altitude_label.config(
                text=f"İrtifa: {telemetry_data['altitude']:.2f} m"
            )
            gps = telemetry_data["gps"]
            if gps and hasattr(gps, "lat") and hasattr(gps, "lon"):
                self.gps_label.config(text=f"GPS: {gps.lat:.6f}, {gps.lon:.6f}")
            else:
                self.gps_label.config(text="GPS: N/A")
            self.battery_label.config(text=f"Pil: {telemetry_data['battery']}%")
        except queue.Empty:
            pass

        try:
            frame = frame_queue.get_nowait()
            # OpenCV BGR formatını RGB'ye çevirip Tkinter uyumlu hale getir
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            self.video_panel.imgtk = imgtk  # Referansı saklayın
            self.video_panel.config(image=imgtk)
        except queue.Empty:
            pass

        self.root.after(30, self.update_gui)


#######################################
# 4. OTONOM VE KONTROLLÜ UÇUŞ FONKSİYONLARI #
#######################################


def arm_and_takeoff(vehicle, target_altitude):
    """
    Drone'u arm eder ve belirtilen yüksekliğe çıkar.
    """
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    print(f"Taking off to {target_altitude} meters")
    vehicle.simple_takeoff(target_altitude)
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_altitude:.2f} meters")
        if current_altitude >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def goto_waypoint(vehicle, latitude, longitude, altitude):
    """
    Drone'u belirtilen GPS koordinatlarına götürür.
    """
    print(f"Going to waypoint: Lat={latitude}, Lon={longitude}, Alt={altitude}")
    target_location = LocationGlobalRelative(latitude, longitude, altitude)
    vehicle.simple_goto(target_location)
    # Hedefe ulaşılana kadar bekle
    while True:
        distance = get_distance_metres(vehicle.location.global_frame, target_location)
        print(f"Distance to waypoint: {distance:.2f} meters")
        if distance <= 1.0:  # 1 metre yakınlıkta kabul et
            print("Reached waypoint")
            break
        time.sleep(1)


def get_distance_metres(location1, location2):
    """
    İki GPS konumu arasındaki mesafeyi hesaplar (metre cinsinden).
    """
    dlat = location2.lat - location1.lat
    dlong = location2.lon - location1.lon
    return np.sqrt(dlat**2 + dlong**2) * 1.113195e5


from pymavlink import mavutil  # Add this line to import mavutil


def send_velocity_command(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Drone'a belirli bir süre boyunca hız komutu gönderir.
    - velocity_x: İleri/geri hız (m/s)
    - velocity_y: Sağ/sol hız (m/s)
    - velocity_z: Yukarı/aşağı hız (m/s)
    - duration: Komutun uygulanacağı süre (saniye)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0,
        0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0,
        0,
        0,  # x, y, z positions (not used)
        velocity_x,
        velocity_y,
        velocity_z,  # x, y, z velocity in m/s
        0,
        0,
        0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0,
        0,  # yaw, yaw_rate (not used)
    )
    for _ in range(int(duration * 10)):  # 10 Hz ile komut gönder
        vehicle.send_mavlink(msg)
        time.sleep(0.1)


def manual_control_gui(root, vehicle):
    """
    Tkinter GUI ile drone'u manuel olarak kontrol etmek için butonlar ekler.
    """
    control_frame = tk.Frame(root)
    control_frame.pack(pady=10)

    def move_forward():
        send_velocity_command(vehicle, 1, 0, 0, 1)  # İleri git

    def move_backward():
        send_velocity_command(vehicle, -1, 0, 0, 1)  # Geri git

    def move_left():
        send_velocity_command(vehicle, 0, -1, 0, 1)  # Sola git

    def move_right():
        send_velocity_command(vehicle, 0, 1, 0, 1)  # Sağa git

    def move_up():
        send_velocity_command(vehicle, 0, 0, -1, 1)  # Yukarı çık

    def move_down():
        send_velocity_command(vehicle, 0, 0, 1, 1)  # Aşağı in

    tk.Button(control_frame, text="İleri", command=move_forward).grid(row=0, column=1)
    tk.Button(control_frame, text="Geri", command=move_backward).grid(row=2, column=1)
    tk.Button(control_frame, text="Sol", command=move_left).grid(row=1, column=0)
    tk.Button(control_frame, text="Sağ", command=move_right).grid(row=1, column=2)
    tk.Button(control_frame, text="Yukarı", command=move_up).grid(row=0, column=0)
    tk.Button(control_frame, text="Aşağı", command=move_down).grid(row=2, column=2)


#######################################
# 5. ANA PROGRAM: BAĞLANTI VE THREADLER #
#######################################


def main():
    """
    Ana fonksiyon:
    Mission Planner SITL ile UDP üzerinden bağlantı kurar.
    Telemetri ve video iş parçacıklarını başlatır.
    Tkinter tabanlı GUI'yi çalıştırır.
    """
    # Mission Planner SITL bağlantısı için UDP bağlantı dizesi (varsayılan)
    connection_string = "tcp:127.0.0.1:5762"
    baud_rate = 57600
    try:
        print("SITL'ye bağlanılıyor...")
        vehicle = connect(
            connection_string, baud=baud_rate, wait_ready=True, timeout=60
        )
    except Exception as e:
        print("SITL bağlantı hatası:", e)
        print("Simülasyon modu başlatılıyor...")
        vehicle = None

    # Telemetri iş parçacığını başlat
    t_telemetry = threading.Thread(
        target=telemetry_thread, args=(vehicle,), daemon=True
    )
    t_telemetry.start()

    # Video akışı ve analiz iş parçacığını başlat
    t_video = threading.Thread(target=video_thread, daemon=True)
    t_video.start()

    # Tkinter GUI'sini başlat
    root = tk.Tk()
    app = GroundStationGUI(root, vehicle)

    # Manuel kontrol butonlarını ekle
    manual_control_gui(root, vehicle)

    # Otonom uçuş için örnek rota
    waypoints = [
        (41.015137, 28.979530, 10),  # Başlangıç noktası
        (41.015200, 28.979600, 10),  # İlk waypoint
        (41.015300, 28.979700, 10),  # İkinci waypoint
    ]

    def start_autonomous_flight():
        print("Otonom uçuş başlatılıyor...")
        arm_and_takeoff(vehicle, 10)
        for wp in waypoints:
            goto_waypoint(vehicle, wp[0], wp[1], wp[2])
        print("Otonom uçuş tamamlandı.")

    tk.Button(root, text="Otonom Uçuş Başlat", command=start_autonomous_flight).pack(
        pady=10
    )

    root.mainloop()


if __name__ == "__main__":
    main()
