#!/usr/bin/env python3
import asyncio
import threading
import time
import tkinter as tk
from tkinter import ttk
import cv2
import numpy as np
from PIL import Image as PilImage, ImageTk
import requests
import uvicorn
from io import BytesIO
import queue
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# DroneKit kütüphanesi (pip install dronekit)
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# FastAPI imports
from fastapi import FastAPI, Query, Response, UploadFile, File
from fastapi.middleware.cors import CORSMiddleware

# -------------------------------------------------
# Stil / Tema Ayarları (Modern Arayüz)
# -------------------------------------------------
def setup_style():
    style = ttk.Style()
    style.theme_use("clam")
    style.configure("TFrame", background="#1E1E1E")
    style.configure(
        "TLabel", background="#1E1E1E", foreground="white", font=("Arial", 10)
    )
    style.configure(
        "TButton",
        background="#4CAF50",
        foreground="white",
        font=("Arial", 10, "bold"),
        borderwidth=0,
    )
    style.map("TButton", background=[("active", "#45a049")])
    style.configure("Header.TLabel", font=("Arial", 14, "bold"), foreground="#FFD700")



# -------------------------------------------------
# Global Asenkron Olay Döngüsü
# -------------------------------------------------
async_loop = asyncio.new_event_loop()
threading.Thread(target=async_loop.run_forever, daemon=True).start()

# ========================================================
# DRONE KONTROL SINIFI (Simülasyon modu)
# ========================================================
class RealDroneController:
    def __init__(self, connection_string="tcp:127.0.0.1:5762"):
        """
        Raspberry Pi 4B üzerinden Pixhawk'a seri bağlantı için örnek bağlantı dizesi.
        Kendi donanımınıza göre güncellenebilir.
        """
        self.connected = False
        self.connection_string = connection_string
        self.vehicle = None
        self.log("Gerçek drone bağlantısı oluşturuluyor...")
        self._connect_drone()

    def log(self, message):
        print(message)

    def _connect_drone(self):
        try:
            self.vehicle = connect(self.connection_string, wait_ready=True, timeout=60)
            self.connected = True
            self.log("Drone bağlantısı başarılı.")
        except Exception as e:
            self.log(f"Drone bağlantı hatası: {e}")
            self.connected = False

    async def arm_and_takeoff(self, target_altitude):
        if not self.connected:
            self.log("Drone bağlı değil. Lütfen bağlantıyı kontrol edin.")
            return
        await asyncio.get_event_loop().run_in_executor(
            None, self._arm_and_takeoff_blocking, target_altitude
        )

    def _arm_and_takeoff_blocking(self, target_altitude):
        vehicle = self.vehicle
        self.log("Arm işlemi başlatılıyor...")
        while not vehicle.is_armable:
            self.log("Drone arm edilebilir durumda değil, bekleniyor...")
            time.sleep(1)
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        while not vehicle.armed:
            self.log("Drone arm oluyor, bekleniyor...")
            time.sleep(1)
        self.log("Drone armed, kalkışa geçiliyor...")
        vehicle.simple_takeoff(target_altitude)
        while True:
            alt = vehicle.location.global_relative_frame.alt
            self.log(f"Mevcut irtifa: {alt:.2f} m")
            if alt >= target_altitude * 0.95:
                break
            time.sleep(1)
        self.log("Hedef irtifaya ulaşıldı.")

    async def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration=1):
        await asyncio.get_event_loop().run_in_executor(
            None,
            self._send_ned_velocity_blocking,
            velocity_x,
            velocity_y,
            velocity_z,
            duration,
        )

    def _send_ned_velocity_blocking(self, velocity_x, velocity_y, velocity_z, duration):
        vehicle = self.vehicle
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Use the correct frame
            0b0000111111000111,  # Only velocity components are active
            0,
            0,
            0,
            velocity_x,
            velocity_y,
            velocity_z,
            0,
            0,
            0,
            0,
            0,
        )
        for _ in range(int(duration * 10)):
            vehicle.send_mavlink(msg)
            vehicle.flush()
            time.sleep(0.1)
        self.log(
            f"{duration} saniye boyunca (x:{velocity_x}, y:{velocity_y}, z:{velocity_z}) hızı gönderildi."
        )

    async def move_3d(
            self,
            velocity_x: float,
            velocity_y: float,
            velocity_z: float,
            duration: float = 1,
    ):
        self.log(
            f"3 boyutlu hareket: x:{velocity_x}, y:{velocity_y}, z:{velocity_z} için {duration} saniye."
        )
        await self.send_ned_velocity(velocity_x, velocity_y, velocity_z, duration)

    async def turn_by_angle(self, angle):
        await asyncio.get_event_loop().run_in_executor(
            None, self._turn_by_angle_blocking, angle
        )

    def _turn_by_angle_blocking(self, angle):
        vehicle = self.vehicle
        self.log(f"{angle:.2f} derece dönme komutu gönderiliyor...")
        msg = vehicle.message_factory.command_long_encode(
            0, 0, 115, 0, angle, 0, 1, 1, 0, 0, 0  # MAV_CMD_CONDITION_YAW
        )
        vehicle.send_mavlink(msg)
        time.sleep(3)
        self.log(f"{angle:.2f} derece dönüş tamamlandı.")

    async def stop(self):
        self.log("Drone durduruluyor...")
        await self.send_ned_velocity(0, 0, 0)

    async def land(self):
        if not self.connected:
            self.log("Drone bağlı değil.")
            return
        await asyncio.get_event_loop().run_in_executor(None, self._land_blocking)

    def _land_blocking(self):
        vehicle = self.vehicle
        self.log("İniş komutu gönderiliyor...")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(10)
        self.log("Drone indi.")

    async def disconnect(self):
        await asyncio.get_event_loop().run_in_executor(None, self._disconnect_blocking)

    def _disconnect_blocking(self):
        if self.vehicle:
            self.log("Drone bağlantısı kesiliyor...")
            self.vehicle.close()
            self.connected = False
            self.log("Drone bağlantısı kesildi.")

    async def move_distance(self, distance):
        self.log(f"Drone {distance:.2f} metre ileri hareket edecek.")
        duration = abs(distance) / 1.0
        await self.send_ned_velocity(1 if distance >= 0 else -1, 0, 0, duration)


class ImageConverter(Node):
    def __init__(self, gui):
        super().__init__('image_converter')
        self.bridge = CvBridge()
        self.gui = gui
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10  # QoS profile depth
        )
        self.subscription  # Prevent unused variable warning

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.gui.root.after(0, self.gui.update_image, cv_image)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')


# -------------------------------------------------
# DroneGUI: Modern Arayüz (Responsive Tasarım)
# -------------------------------------------------
class DroneGUI:
    def __init__(self, root, controller):
        self.root = root
        self.controller = controller
        self.connected = False
        self.root.title("IHA Kontrol Paneli (Gerçek Drone - Pixhawk / Raspberry Pi)")
        self.root.geometry("1200x800")
        self.root.resizable(True, True)
        setup_style()

        self.mode = "autonomous"
        self.main_frame = ttk.Frame(root, padding=10)
        self.main_frame.grid(row=0, column=0, sticky="nsew")
        root.rowconfigure(0, weight=1)
        root.columnconfigure(0, weight=1)

        self.header_frame = ttk.Frame(self.main_frame)
        self.header_frame.grid(row=0, column=0, columnspan=5, sticky="ew", pady=(0, 10))
        self.header_label = ttk.Label(
            self.header_frame, text="Drone Kontrol Paneli", style="Header.TLabel"
        )
        self.header_label.pack(fill="x")

        self.image_label = ttk.Label(self.header_frame)
        self.image_label.pack()

        self.button_frame = ttk.Frame(self.main_frame)
        self.button_frame.grid(row=1, column=0, columnspan=5, sticky="ew", pady=(0, 10))
        self.connect_button = ttk.Button(
            self.button_frame, text="Drone Bağlan", width=20, command=self.connect_drone
        )
        self.connect_button.grid(row=0, column=0, padx=5, pady=5)
        self.manual_button = ttk.Button(
            self.button_frame,
            text="Manuel Mod",
            width=20,
            command=self.start_manual,
            state="disabled",
        )
        self.manual_button.grid(row=0, column=1, padx=5, pady=5)
        self.auto_button = ttk.Button(
            self.button_frame,
            text="Otonom Mod",
            width=20,
            command=self.start_autonomous,
            state="disabled",
        )
        self.auto_button.grid(row=0, column=2, padx=5, pady=5)
        self.land_button = ttk.Button(
            self.button_frame,
            text="İniş",
            width=20,
            command=self.land,
            state="disabled",
        )
        self.land_button.grid(row=0, column=3, padx=5, pady=5)
        self.disconnect_button = ttk.Button(
            self.button_frame,
            text="Drone Bağlantısını Kes",
            width=20,
            command=self.disconnect_drone,
            state="disabled",
        )
        self.disconnect_button.grid(row=0, column=4, padx=5, pady=5)

        self.video_frame = ttk.Frame(self.main_frame, width=640, height=480)
        self.video_frame.grid(
            row=2, column=0, columnspan=3, sticky="nsew", pady=(0, 10)
        )
        self.video_frame.grid_propagate(False)
        self.video_label = ttk.Label(self.video_frame)
        self.video_label.pack(fill="none", expand=True)

        self.manual_frame = ttk.Frame(self.main_frame)
        self._build_manual_panel()
        self.manual_frame.grid(row=2, column=3, sticky="nsew", padx=10, pady=10)
        self.manual_frame.grid_forget()

        self.info_frame = ttk.Frame(self.main_frame)
        self.info_frame.grid(row=3, column=0, columnspan=5, sticky="ew", pady=(0, 10))
        self.info_text = tk.Text(
            self.info_frame, height=10, width=100, bg="#1E1E1E", fg="white"
        )
        self.info_text.pack(fill="both", expand=True)
        self.log("Drone kontrol paneli başlatıldı...")

        self.telemetry_frame = ttk.Frame(self.main_frame)
        self.telemetry_frame.grid(
            row=4, column=0, columnspan=5, sticky="ew", pady=(0, 10)
        )
        self.telemetry_label = ttk.Label(
            self.telemetry_frame, text="Telemetri: Bekleniyor...", font=("Courier", 10)
        )
        self.telemetry_label.pack(anchor="w")

        self.root.bind("<Configure>", self.on_resize)
        self.root.after(1000, self.check_intersection_api)
        self.root.after(1000, self.check_image_analysis_api)
        self.root.after(1000, self.check_path_api)
        self.root.after(2000, self.check_crowd_api)
        self.root.after(30000, self.check_traffic_analysis_api)
        self.update_telemetry()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def update_image(self, cv_image):
        try:
            image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            image = PilImage.fromarray(image)  # Doğru şekilde çağır
            image = image.resize((640, 480), PilImage.LANCZOS)  # ANTIALIAS yerine LANCZOS kullan
            
            imgtk = ImageTk.PhotoImage(image=image)
            self.image_label.config(image=imgtk)
            self.image_label.image = imgtk  # Referansı koru
        except Exception as e:
            print(f"Görüntü işleme hatası: {e}")



    def on_resize(self, event):
        try:
            imgtk = getattr(self.video_label, "imgtk", None)
            if imgtk:
                image = ImageTk.getimage(imgtk)
                resized_image = image.resize(
                    (event.width - 20, event.height - 20), Image.ANTIALIAS
                )
                new_imgtk = ImageTk.PhotoImage(resized_image)
                self.video_label.config(image=new_imgtk)
                self.video_label.imgtk = new_imgtk
        except Exception:
            pass

    def log(self, message):
        self.info_text.insert(tk.END, message + "\n")
        self.info_text.see(tk.END)
        print(message)

    def _build_manual_panel(self):
        directions = [
            ("⬆️", "forward", 0, 1),
            ("⬅️", "left", 1, 0),
            ("⬇️", "backward", 2, 1),
            ("➡️", "right", 1, 2),
            ("↖️", "left_forward", 0, 0),
            ("↗️", "right_forward", 0, 2),
            ("↙️", "left_backward", 2, 0),
            ("↘️", "right_backward", 2, 2),
        ]
        for emoji, dir_name, r, c in directions:
            btn = ttk.Button(
                self.manual_frame,
                text=emoji,
                width=5,
                command=lambda d=dir_name: self.manual_control(d),
            )
            btn.grid(row=r, column=c, padx=3, pady=3)

        up_button = ttk.Button(
            self.manual_frame,
            text="⬆️ UP",
            width=10,
            command=lambda: self.manual_control("up"),
        )
        up_button.grid(row=0, column=4, padx=40, pady=3)

        down_button = ttk.Button(
            self.manual_frame,
            text="⬇️ DOWN",
            width=10,
            command=lambda: self.manual_control("down"),
        )
        down_button.grid(row=2, column=4, padx=40, pady=3)

    # -----------------------
    # Drone Kontrol Metotları
    # -----------------------
    def connect_drone(self):
        if self.connected:
            self.log("Drone zaten bağlı.")
            return
        self.log("Drone bağlantısı sağlanıyor...")
        if not self.controller.connected:
            self.controller = RealDroneController(connection_string="tcp:127.0.0.1:5762")
        if self.controller.connected:
            self.connected = True
            self.log("Drone bağlantısı sağlandı.")
            self.manual_button.config(state="normal")
            self.auto_button.config(state="normal")
            self.land_button.config(state="normal")
            self.disconnect_button.config(state="normal")
        else:
            self.log("Drone bağlantısı yapılamadı.")

    def disconnect_drone(self):
        if not self.connected:
            self.log("Drone zaten bağlantısız.")
            return
        self.log("Drone bağlantısı kesiliyor...")
        asyncio.run_coroutine_threadsafe(self.controller.disconnect(), async_loop)
        self.connected = False
        self.manual_button.config(state="disabled")
        self.auto_button.config(state="disabled")
        self.land_button.config(state="disabled")
        self.disconnect_button.config(state="disabled")
        self.log("Drone bağlantısı kesildi.")

    def start_manual(self):
        if not self.connected:
            self.log("Lütfen önce drone bağlantısını kurun.")
            return
        self.mode = "manual"
        self.log("Manuel mod aktif.")
        self.manual_frame.grid(row=2, column=3, sticky="nsew", padx=10, pady=10)
        asyncio.run_coroutine_threadsafe(self.controller.arm_and_takeoff(5), async_loop)

    def start_autonomous(self):
        if not self.connected:
            self.log("Lütfen önce drone bağlantısını kurun.")
            return
        self.mode = "autonomous"
        self.log("Otonom mod aktif.")
        self.manual_frame.grid_forget()

    def manual_control(self, direction):
        if not self.connected:
            self.log("Lütfen önce drone bağlantısını kurun.")
            return
        self.log(f"Manuel kontrol: {direction}")
        mapping = {
            "forward": (1, 0, 0),
            "backward": (-1, 0, 0),
            "left": (0, -1, 0),
            "right": (0, 1, 0),
            "up": (0, 0, -1),  # DroneKit'te Z ekseni ters yönde yukarıdır.
            "down": (0, 0, 1),
            "left_forward": (1, -1, 0),
            "right_forward": (1, 1, 0),
            "left_backward": (-1, -1, 0),
            "right_backward": (-1, 1, 0),
        }
        if direction in mapping:
            vx, vy, vz = mapping.get(direction)
            asyncio.run_coroutine_threadsafe(
                self.controller.move_3d(vx, vy, vz, duration=1), async_loop
            )
        else:
            self.log(f"Bilinmeyen yön: {direction}")

    def land(self):
        if not self.connected:
            self.log("Lütfen önce drone bağlantısını kurun.")
            return
        self.log("İniş komutu gönderildi.")
        asyncio.run_coroutine_threadsafe(self.controller.land(), async_loop)

    def on_closing(self):
        asyncio.run_coroutine_threadsafe(self.controller.disconnect(), async_loop)
        self.root.destroy()

    # -----------------------
    # Video ve Telemetri Güncellemeleri
    # -----------------------
    def update_video(self):
        def fetch_and_update():
            try:
                response = requests.get(
                    "http://localhost:5000/camera_feed", timeout=0.5
                )
                if response.status_code == 200:
                    img = Image.open(BytesIO(response.content))
                    img = img.resize((640, 480), Image.LANCZOS)
                    imgtk = ImageTk.PhotoImage(img)
                    self.video_label.after(
                        0, lambda: self.video_label.configure(image=imgtk)
                    )
                    self.video_label.imgtk = imgtk
                elif response.status_code == 503:
                    self.log("Kamera akışı bekleniyor...")
                else:
                    self.log(f"Kamera hatası: {response.status_code}")
            except Exception as e:
                self.log(f"Görüntü alınamadı: {str(e)}")

        threading.Thread(target=fetch_and_update, daemon=True).start()
        self.root.after(50, self.update_video)

    def update_telemetry(self):
        try:
            loc = self.controller.vehicle.location.global_frame
            alt = self.controller.vehicle.location.global_relative_frame.alt
            gps = f"Lat: {loc.lat:.6f}, Lon: {loc.lon:.6f}, Alt: {alt:.2f} m"
        except Exception:
            gps = "GPS bilgisi alınamadı."
        try:
            bat = self.controller.vehicle.battery
            battery = f"Voltage: {bat.voltage} V, Current: {bat.current} A, Level: {bat.level} %"
        except Exception:
            battery = "Batarya bilgisi alınamadı."
        try:
            att = self.controller.vehicle.attitude
            attitude = (
                f"Roll: {att.roll:.2f}, Pitch: {att.pitch:.2f}, Yaw: {att.yaw:.2f}"
            )
        except Exception:
            attitude = "Attitude bilgisi alınamadı."
        telemetry_str = (
            f"Control Mode: {self.mode}\n"
            f"GPS: {gps}\n"
            f"Batarya: {battery}\n"
            f"Attitude: {attitude}\n"
        )
        self.telemetry_label.config(text=telemetry_str)
        self.root.after(1000, self.update_telemetry)

    # -----------------------
    # Arka Plan API Çağrıları (Otonom Mod)
    # -----------------------
    def check_intersection_api(self):
        if not self.connected or self.mode != "autonomous":
            self.root.after(1000, self.check_intersection_api)
            return

        def fetch():
            try:
                response = requests.get(
                    "http://10.225.217.213:8000/intersection", timeout=1
                )
                if response.status_code == 200:
                    data = response.json()
                    if data.get("is_intersection", False):
                        self.root.after(
                            0,
                            lambda: self.log(
                                "4’lü kavşak tespit edildi! Drone durduruluyor..."
                            ),
                        )
                        asyncio.run_coroutine_threadsafe(
                            self.controller.stop(), async_loop
                        )
                        details_response = requests.get(
                            "http://localhost:5000/intersection_details", timeout=1
                        )
                        if details_response.status_code == 200:
                            details = details_response.json()
                            angle = details.get("angle")
                            distance = details.get("distance")
                            self.root.after(
                                0,
                                lambda: self.log(
                                    f"Kavşak detayları: Açısı = {angle:.2f}, Mesafe = {distance:.2f}"
                                ),
                            )
                            asyncio.run_coroutine_threadsafe(
                                self.controller.turn_by_angle(angle), async_loop
                            )
                            asyncio.run_coroutine_threadsafe(
                                self.controller.move_distance(distance), async_loop
                            )
                        else:
                            self.root.after(
                                0, lambda: self.log("Intersection details API hatalı.")
                            )
            except Exception as e:
                self.root.after(0, lambda e=e: self.log(f"Intersection API hatası: {e}"))

        threading.Thread(target=fetch, daemon=True).start()
        self.root.after(1000, self.check_intersection_api)

    def check_image_analysis_api(self):
        if not self.connected or self.mode != "autonomous":
            self.root.after(1000, self.check_image_analysis_api)
            return

        def fetch():
            try:
                response = requests.get(
                    "http://localhost:5000/image_analysis", timeout=1
                )
                if response.status_code == 200:
                    data = response.json()
                    analysis_info = f"Görüntü Analizi: {data.get('analysis')}, Sayım: {data.get('count')}"
                    self.root.after(0, lambda: self.log(analysis_info))
            except Exception as e:
                self.root.after(0, lambda: self.log(f"Görüntü analiz API hatası: {e}"))

        threading.Thread(target=fetch, daemon=True).start()
        self.root.after(1000, self.check_image_analysis_api)

    def check_crowd_api(self):
        if not self.connected or self.mode != "autonomous":
            self.root.after(2000, self.check_crowd_api)
            return

        def fetch():
            try:
                response = requests.get(
                    "http://localhost:5000/crowd_details", timeout=1
                )
                if response.status_code == 200:
                    data = response.json()
                    if data.get("crowd_found", False):
                        angle = data.get("angle")
                        distance = data.get("distance")
                        self.root.after(
                            0,
                            lambda: self.log(
                                f"Kalabalık alan tespit edildi: Açısı = {angle:.2f}, Mesafe = {distance:.2f}"
                            ),
                        )
                        asyncio.run_coroutine_threadsafe(
                            self.controller.turn_by_angle(angle), async_loop
                        )
                        asyncio.run_coroutine_threadsafe(
                            self.controller.move_distance(distance), async_loop
                        )
            except Exception as e:
                self.root.after(0, lambda: self.log(f"Crowd API hatası: {e}"))

        threading.Thread(target=fetch, daemon=True).start()
        self.root.after(2000, self.check_crowd_api)

    def check_path_api(self):
        if not self.connected or self.mode != "autonomous":
            self.root.after(1000, self.check_path_api)
            return

        def fetch():
            try:
                response = requests.get(
                    "http://localhost:5000/path_direction", timeout=1
                )
                if response.status_code == 200:
                    data = response.json()
                    angle = data.get("angle")
                    distance = data.get("distance")
                    self.root.after(
                        0,
                        lambda: self.log(
                            f"Yol tespiti: Açısı = {angle:.2f}, Mesafe = {distance:.2f}"
                        ),
                    )
                    asyncio.run_coroutine_threadsafe(
                        self.controller.turn_by_angle(angle), async_loop
                    )
                    asyncio.run_coroutine_threadsafe(
                        self.controller.move_distance(distance), async_loop
                    )
            except Exception as e:
                self.root.after(0, lambda: self.log(f"Path API hatası: {e}"))

        threading.Thread(target=fetch, daemon=True).start()
        self.root.after(1000, self.check_path_api)

    def check_traffic_analysis_api(self):
        if not self.connected or self.mode != "autonomous":
            self.root.after(30000, self.check_traffic_analysis_api)
            return

        def fetch():
            try:
                response = requests.get(
                    "http://localhost:5000/analyze_traffic", timeout=2
                )
                if response.status_code == 200:
                    data = response.json()
                    busiest_road = data.get("busiest_road")
                    vehicle_count = data.get("vehicle_count")
                    pedestrian_count = data.get("pedestrian_count")
                    self.root.after(
                        0,
                        lambda: self.log(
                            f"Trafik Analizi: En kalabalık yol = {busiest_road}, Araç = {vehicle_count}, Yaya = {pedestrian_count}"
                        ),
                    )
                    opt_response = requests.get(
                        "http://localhost:5000/optimize_traffic_lights", timeout=2
                    )
                    if opt_response.status_code == 200:
                        opt_data = opt_response.json()
                        self.root.after(
                            0,
                            lambda: self.log(
                                f"Akıllı Trafik Işığı: {opt_data.get('status')}"
                            ),
                        )
                    else:
                        self.root.after(
                            0, lambda: self.log("Trafik analizi API hatalı.")
                        )
            except Exception as e:
                self.root.after(0, lambda: self.log(f"Trafik analizi API hatası: {e}"))

        threading.Thread(target=fetch, daemon=True).start()
        self.root.after(30000, self.check_traffic_analysis_api)

# -------------------------------------------------
# FastAPI Uç Noktaları (Gerçek Drone ve Raspberry Pi)
# -------------------------------------------------
app = FastAPI()

# CORS ayarları: Eğer mobil uygulama farklı bir cihazdan erişecekse gerekli
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pixhawk bağlantısı için Raspberry Pi üzerinden seri bağlantı dizesi kullanılıyor.
global_controller = RealDroneController(
    connection_string="tcp:127.0.0.1:5762"
)


# Mevcut Drone Kontrol API’leri
@app.get("/connect")
async def api_connect():
    return {"status": "Gerçek drone connected"}


@app.get("/arm_takeoff")
async def api_arm_takeoff(altitude: int = 10):
    await global_controller.arm_and_takeoff(altitude)
    return {"status": f"Drone {altitude} metreye çıktı"}


@app.get("/move")
async def api_move(
    direction: str = Query(
        ...,
        pattern="^(forward|backward|left|right|up|down|left_forward|right_forward|left_backward|right_backward)$", #regex yazıyordu pattern diye değiştirildi.
    )
):
    return {"status": f"Drone {direction} hareket etti (dummy)"}


@app.get("/move3d")
async def api_move3d(
    velocity_x: float, velocity_y: float, velocity_z: float, duration: float = 1
):
    await global_controller.move_3d(velocity_x, velocity_y, velocity_z, duration)
    return {"status": "Drone 3 boyutlu hareket gerçekleştirdi"}


@app.get("/turn")
async def api_turn(angle: float):
    await global_controller.turn_by_angle(angle)
    return {"status": f"Drone {angle} derece döndü"}


@app.get("/move_distance")
async def api_move_distance(distance: float):
    await global_controller.move_distance(distance)
    return {"status": f"Drone {distance} metre ilerledi"}


@app.get("/stop")
async def api_stop():
    await global_controller.stop()
    return {"status": "Drone durdu"}


@app.get("/land")
async def api_land():
    await global_controller.land()
    return {"status": "Drone indi"}


@app.get("/disconnect")
async def api_disconnect():
    await global_controller.disconnect()
    return {"status": "Drone disconnected"}


@app.get("/telemetry")
async def api_telemetry():
    try:
        loc = global_controller.vehicle.location.global_frame
        alt = global_controller.vehicle.location.global_relative_frame.alt
        gps = {"lat": loc.lat, "lon": loc.lon, "alt": alt}
    except Exception:
        gps = {}
    try:
        bat = global_controller.vehicle.battery
        battery = {"voltage": bat.voltage, "current": bat.current, "level": bat.level}
    except Exception:
        battery = {}
    try:
        att = global_controller.vehicle.attitude
        attitude = {"roll": att.roll, "pitch": att.pitch, "yaw": att.yaw}
    except Exception:
        attitude = {}
    return {"gps": gps, "battery": battery, "attitude": attitude}


# Kamera akışı için uç noktalar
frame_queue = queue.Queue(maxsize=50)


@app.post("/upload_frame")
async def upload_frame(file: UploadFile = File(...)):
    try:
        contents = await file.read()
        if frame_queue.full():
            frame_queue.get()
        frame_queue.put(contents)
        return {"status": "Frame alındı"}
    except Exception as e:
        return {"error": str(e)}


@app.get("/camera_feed")
async def api_camera_feed():
    try:
        if not frame_queue.empty():
            frame_data = frame_queue.get()
            print(f"Kuyruk durumu: {frame_queue.qsize()} frame kaldı.")
            return Response(content=frame_data, media_type="image/jpeg")
        else:
            print("Kuyruk boş, frame alınamadı.")
            return Response(status_code=503)
    except Exception as e:
        print(f"Hata: {e}")
        return Response(status_code=500)


# -------------------------------
# Mobil Uygulama İçin Ek API Uç Noktaları
# -------------------------------
# Bu uç noktalar mobil uygulamada yer alan sayaç (counter) gibi basit komutları destekleyecek.
counter_value = 0


@app.get("/counter")
def get_counter():
    global counter_value
    return {"counter": counter_value}


@app.post("/counter/increment")
def increment_counter():
    global counter_value
    counter_value += 1
    return {"counter": counter_value}


def run_api():
    uvicorn.run(app, host="0.0.0.0", port=5000)


# -------------------------------------------------
# Ana Program: Hem GUI hem de API aynı anda çalışıyor (Gerçek Drone - Pixhawk & Raspberry Pi)
# -------------------------------------------------
if __name__ == "__main__":
    # API sunucusunu ayrı bir iş parçacığında başlatıyoruz.
    api_thread = threading.Thread(target=run_api, daemon=True)
    api_thread.start()

    # Tkinter tabanlı GUI başlatılıyor.
    rclpy.init()
    root = tk.Tk()
    gui_controller = global_controller
    app_gui = DroneGUI(root, gui_controller)
    image_converter = ImageConverter(app_gui)
    def ros_spin():
        rclpy.spin(image_converter)
    threading.Thread(target=ros_spin, daemon=True).start()
    # Eğer gerçek video akışı varsa update_video() metodunun yorumunu kaldırabilirsiniz.
    # app_gui.update_video()
    root.mainloop()
    rclpy.shutdown()