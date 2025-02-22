#!/usr/bin/env python3
import tkinter as tk
import cv2
from PIL import Image, ImageTk
import time
import requests

# Gerçek sensör kütüphaneleri
import Adafruit_DHT
import board
import busio
import adafruit_bmp280
import smbus2

# Gerçek drone kontrolü için DroneKit kullanımı
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil


# --------------------------------------------------------
# Gerçek Sensör Entegrasyonu (AdditionalSensors)
# --------------------------------------------------------
class AdditionalSensors:
    def __init__(self):
        # DHT22: Sıcaklık ve nem için
        self.DHT_SENSOR = Adafruit_DHT.DHT22
        self.DHT_PIN = 4  # Raspberry Pi GPIO4 örneği

        # BMP280: Basınç sensörü
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)

        # MPU6050: İvmeölçer
        self.bus = smbus2.SMBus(1)
        self.mpu_addr = 0x68
        self.bus.write_byte_data(self.mpu_addr, 0x6B, 0)  # Uyandırma

        # Ağırlık sensörü (HX711) ve LiDAR sensörü için placeholder fonksiyonlar:
        # Bu fonksiyonları kullandığınız donanıma uygun şekilde implement edin.

    def read_weight(self):
        try:
            weight = read_weight_sensor()  # Kendi HX711 okuma fonksiyonunuzu yazın.
            return weight
        except Exception as e:
            return None

    def read_temperature_and_humidity(self):
        humidity, temperature = Adafruit_DHT.read_retry(self.DHT_SENSOR, self.DHT_PIN)
        return temperature, humidity

    def read_pressure(self):
        return self.bmp280.pressure

    def read_acceleration(self):
        def read_word(reg):
            high = self.bus.read_byte_data(self.mpu_addr, reg)
            low = self.bus.read_byte_data(self.mpu_addr, reg + 1)
            value = (high << 8) + low
            if value >= 0x8000:
                value = -((65535 - value) + 1)
            return value

        accel_x = read_word(0x3B) / 16384.0
        accel_y = read_word(0x3D) / 16384.0
        accel_z = read_word(0x3F) / 16384.0
        return accel_x, accel_y, accel_z

    def read_lidar_distance(self):
        try:
            distance = read_lidar_sensor()  # Kendi LiDAR okuma fonksiyonunuzu yazın.
            return distance
        except Exception as e:
            return None


# --------------------------------------------------------
# Gerçek Drone Kontrolü (DroneController)
# --------------------------------------------------------
class DroneController:
    def __init__(self, connection_string="/dev/ttyAMA0", baud=57600):
        print("Gerçek drone bağlantısı kuruluyor...")
        self.vehicle = connect(connection_string, baud=baud, wait_ready=True)

    def connect_drone(self):
        # DroneKit bağlantısı __init__ sırasında sağlanmıştır.
        print("Drone bağlantısı başarılı.")

    def arm_and_takeoff(self, target_altitude):
        print("Drone arm edilebilir durumda mı kontrol ediliyor...")
        while not self.vehicle.is_armable:
            print("Drone arm edilebilir durumda değil...")
            time.sleep(1)
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("Drone arm ediliyor...")
            time.sleep(1)
        print("Kalkış yapılıyor...")
        self.vehicle.simple_takeoff(target_altitude)
        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            print("Mevcut Yükseklik:", alt)
            if alt >= target_altitude * 0.95:
                print("Hedef yüksekliğe ulaşıldı.")
                break
            time.sleep(1)

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration=1):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0,
            0,
            0b0000111111000111,  # Sadece hız bileşenlerini aktif
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
        for _ in range(int(duration)):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

    def move(self, direction):
        # Manuel kontrol için diagonal (ara) yönler de eklenmiştir.
        if direction == "forward":
            self.send_ned_velocity(1, 0, 0)
        elif direction == "backward":
            self.send_ned_velocity(-1, 0, 0)
        elif direction == "left":
            self.send_ned_velocity(0, -1, 0)
        elif direction == "right":
            self.send_ned_velocity(0, 1, 0)
        elif direction == "up":
            self.send_ned_velocity(0, 0, -1)
        elif direction == "down":
            self.send_ned_velocity(0, 0, 1)
        elif direction == "up_left":
            self.send_ned_velocity(1, -1, 0)
        elif direction == "up_right":
            self.send_ned_velocity(1, 1, 0)
        elif direction == "down_left":
            self.send_ned_velocity(-1, -1, 0)
        elif direction == "down_right":
            self.send_ned_velocity(-1, 1, 0)
        else:
            print("Bilinmeyen hareket komutu:", direction)

    def turn_by_angle(self, angle):
        # Yaw kontrolü: Göreceli dönüş
        is_relative = 1  # göreceli dönüş
        msg = self.vehicle.message_factory.command_long_encode(
            0,
            0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            angle,  # hedef açı
            10,     # hız (derece/s)
            1,      # yön (1 = saat yönünde)
            is_relative,
            0,
            0,
            0,
        )
        self.vehicle.send_mavlink(msg)
        time.sleep(3)

    def move_distance(self, distance):
        # 1 m/s hız varsayımıyla ileri hareket
        self.send_ned_velocity(1, 0, 0, duration=distance)

    def stop(self):
        self.send_ned_velocity(0, 0, 0)

    def land(self):
        print("İniş komutu gönderiliyor...")
        self.vehicle.mode = VehicleMode("LAND")
        while self.vehicle.armed:
            print("Drone iniyor, mevcut yükseklik:", self.vehicle.location.global_relative_frame.alt)
            time.sleep(1)
        print("Drone indi.")

    def disconnect(self):
        self.vehicle.close()
        print("Drone bağlantısı kesildi.")


# --------------------------------------------------------
# Drone GUI, Sürüş Algoritmaları ve Telemetri
# --------------------------------------------------------
class DroneGUI:
    def __init__(self, root, controller):
        self.root = root
        self.controller = controller
        self.root.title("IHA Kontrol Paneli")
        self.mode = None  # "autonomous" veya "manual"

        self.additional_sensors = AdditionalSensors()

        # Video akışı alanı
        self.video_label = tk.Label(root)
        self.video_label.grid(row=0, column=0, columnspan=4, padx=10, pady=10)

        # Üst kontrol butonları
        self.connect_button = tk.Button(root, text="Drone Bağlan", width=20, command=self.connect_drone)
        self.connect_button.grid(row=1, column=0, padx=5, pady=5)

        self.autonomous_button = tk.Button(root, text="Otonom Sürüş Başlat", width=20, command=self.start_autonomous)
        self.autonomous_button.grid(row=1, column=1, padx=5, pady=5)

        self.manual_button = tk.Button(root, text="Manuel Sürüş Başlat", width=20, command=self.start_manual)
        self.manual_button.grid(row=1, column=2, padx=5, pady=5)

        self.stop_button = tk.Button(root, text="Durdur", width=20, command=self.stop)
        self.stop_button.grid(row=1, column=3, padx=5, pady=5)

        # Alt kontrol butonları
        self.land_button = tk.Button(root, text="İniş", width=20, command=self.land)
        self.land_button.grid(row=4, column=0, padx=5, pady=5)

        self.disconnect_button = tk.Button(root, text="Drone Bağlantısını Kes", width=20, command=self.disconnect_drone)
        self.disconnect_button.grid(row=4, column=1, padx=5, pady=5)

        # Manuel kontrol: 3x3 ızgara (merkez boş bırakıldı)
        self.manual_frame = tk.Frame(root)
        self.manual_frame.grid(row=2, column=0, columnspan=4, padx=10, pady=10)
        self.btn_up_left = tk.Button(self.manual_frame, text="↖", width=5, command=lambda: self.manual_control("up_left"))
        self.btn_up = tk.Button(self.manual_frame, text="↑", width=5, command=lambda: self.manual_control("up"))
        self.btn_up_right = tk.Button(self.manual_frame, text="↗", width=5, command=lambda: self.manual_control("up_right"))
        self.btn_left = tk.Button(self.manual_frame, text="←", width=5, command=lambda: self.manual_control("left"))
        self.btn_right = tk.Button(self.manual_frame, text="→", width=5, command=lambda: self.manual_control("right"))
        self.btn_down_left = tk.Button(self.manual_frame, text="↙", width=5, command=lambda: self.manual_control("down_left"))
        self.btn_down = tk.Button(self.manual_frame, text="↓", width=5, command=lambda: self.manual_control("down"))
        self.btn_down_right = tk.Button(self.manual_frame, text="↘", width=5, command=lambda: self.manual_control("down_right"))

        # 3x3 ızgara yerleşimi (merkez boş)
        self.btn_up_left.grid(row=0, column=0, padx=5, pady=5)
        self.btn_up.grid(row=0, column=1, padx=5, pady=5)
        self.btn_up_right.grid(row=0, column=2, padx=5, pady=5)
        self.btn_left.grid(row=1, column=0, padx=5, pady=5)
        # Merkez hücre boş bırakıldı.
        self.btn_right.grid(row=1, column=2, padx=5, pady=5)
        self.btn_down_left.grid(row=2, column=0, padx=5, pady=5)
        self.btn_down.grid(row=2, column=1, padx=5, pady=5)
        self.btn_down_right.grid(row=2, column=2, padx=5, pady=5)
        self.hide_manual_controls()

        # Bilgi & log alanı
        self.info_text = tk.Text(root, height=10, width=80)
        self.info_text.grid(row=3, column=0, columnspan=4, padx=10, pady=10)
        self.info_text.insert(tk.END, "Sistem başlatıldı...\n")

        # Telemetri alanı
        self.telemetry_label = tk.Label(root, text="Telemetri: Bekleniyor...", justify="left", font=("Courier", 10))
        self.telemetry_label.grid(row=5, column=0, columnspan=4, padx=10, pady=10, sticky="w")

        # Video yakalama (kamera)
        self.cap = cv2.VideoCapture(0)
        self.update_video()
        self.update_telemetry()

    def connect_drone(self):
        self.info_text.insert(tk.END, "Drone bağlantısı kuruluyor...\n")
        self.controller.connect_drone()
        self.info_text.insert(tk.END, "Drone bağlandı.\n")

    def disconnect_drone(self):
        self.info_text.insert(tk.END, "Drone bağlantısı kesiliyor...\n")
        self.controller.disconnect()
        self.info_text.insert(tk.END, "Drone bağlantısı kesildi.\n")

    def hide_manual_controls(self):
        self.manual_frame.grid_remove()

    def show_manual_controls(self):
        self.manual_frame.grid()

    def manual_control(self, direction):
        self.info_text.insert(tk.END, f"Manuel kontrol: {direction}\n")
        self.controller.move(direction)

    def update_video(self):
        ret, frame = self.cap.read()
        if ret:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=image)
            self.video_label.imgtk = imgtk
            self.video_label.configure(image=imgtk)
            if self.mode == "autonomous":
                self.controller.move("forward")
        self.root.after(100, self.update_video)

    def update_telemetry(self):
        temperature, humidity = self.additional_sensors.read_temperature_and_humidity()
        pressure = self.additional_sensors.read_pressure()
        accel = self.additional_sensors.read_acceleration()
        weight = self.additional_sensors.read_weight()
        lidar = self.additional_sensors.read_lidar_distance()
        telemetry_str = f"Control Mode: {self.mode if self.mode is not None else 'None'}\n"
        telemetry_str += f"Temperature: {temperature:.2f} °C, Humidity: {humidity:.2f} %\n"
        telemetry_str += f"Pressure: {pressure:.2f} hPa\n"
        telemetry_str += f"Acceleration: ax: {accel[0]:.2f}, ay: {accel[1]:.2f}, az: {accel[2]:.2f}\n"
        telemetry_str += f"Weight: {weight}\n"
        telemetry_str += f"Lidar Distance: {lidar}\n"
        self.telemetry_label.config(text=telemetry_str)
        self.root.after(1000, self.update_telemetry)

    def start_autonomous(self):
        self.mode = "autonomous"
        self.hide_manual_controls()
        self.info_text.insert(tk.END, "Otonom sürüş başlatıldı...\n")
        try:
            # İlk olarak 50 metre yüksekliğe kalkış
            self.controller.arm_and_takeoff(50)
            # Mevcut check_intersection ve check_crowd fonksiyonlarını çalıştırmaya devam ediyoruz
            self.check_intersection_api()
            self.check_crowd_api()
            # Ek olarak sürekli yol tespiti için check_path_api çalıştırılıyor
            self.check_path_api()
        except Exception as e:
            self.info_text.insert(tk.END, f"Kalkış sırasında hata: {e}\n")

    def start_manual(self):
        self.mode = "manual"
        self.show_manual_controls()
        self.info_text.insert(tk.END, "Manuel sürüş başlatıldı...\n")
        try:
            self.controller.arm_and_takeoff(5)
        except Exception as e:
            self.info_text.insert(tk.END, f"Kalkış sırasında hata: {e}\n")

    def stop(self):
        self.mode = None
        self.hide_manual_controls()
        self.info_text.insert(tk.END, "Sistem durduruldu.\n")
        self.controller.stop()

    def land(self):
        self.mode = None
        self.hide_manual_controls()
        self.info_text.insert(tk.END, "İniş komutu gönderildi.\n")
        self.controller.land()

    def on_closing(self):
        self.cap.release()
        self.controller.disconnect()
        self.root.destroy()

    def check_intersection_api(self):
        if self.mode != "autonomous":
            self.root.after(1000, self.check_intersection_api)
            return
        try:
            response = requests.get("http://localhost:5000/check_intersection", timeout=1)
            if response.status_code == 200:
                data = response.json()
                if data.get("four_way_intersection", False):
                    self.info_text.insert(tk.END, "4'lü kavşak tespit edildi! Drone durduruluyor...\n")
                    self.controller.stop()
                    details_response = requests.get("http://localhost:5000/intersection_details", timeout=1)
                    if details_response.status_code == 200:
                        details = details_response.json()
                        angle = details.get("angle")
                        distance = details.get("distance")
                        self.info_text.insert(tk.END, f"Kavşak merkezi detayları: Açısı = {angle:.2f}, Mesafe = {distance:.2f}\n")
                        self.controller.turn_by_angle(angle)
                        self.controller.move_distance(distance)
                    else:
                        self.info_text.insert(tk.END, "Intersection details API hatalı.\n")
        except Exception as e:
            self.info_text.insert(tk.END, f"Intersection API hatası: {e}\n")
        self.root.after(1000, self.check_intersection_api)

    def check_crowd_api(self):
        if self.mode != "autonomous":
            self.root.after(2000, self.check_crowd_api)
            return
        try:
            response = requests.get("http://localhost:5000/crowd_details", timeout=1)
            if response.status_code == 200:
                data = response.json()
                if data.get("crowd_found", False):
                    angle = data.get("angle")
                    distance = data.get("distance")
                    self.info_text.insert(tk.END, f"Kalabalık alan tespit edildi: Açısı = {angle:.2f}, Mesafe = {distance:.2f}. Drone yönlendiriliyor.\n")
                    self.controller.turn_by_angle(angle)
                    self.controller.move_distance(distance)
        except Exception as e:
            self.info_text.insert(tk.END, f"Crowd details API hatası: {e}\n")
        self.root.after(2000, self.check_crowd_api)

    def check_path_api(self):
        """
        Otonom moddayken, her saniye backend'e "/path_direction" isteği gönderilir.
        Bu endpoint, drone'un takip etmesi gereken yolun yönü (açı) ve ilerleme mesafesi bilgilerini döndürür.
        Alınan değerlere göre drone yönlendirilir.
        """
        if self.mode != "autonomous":
            self.root.after(1000, self.check_path_api)
            return

        try:
            response = requests.get("http://localhost:5000/path_direction", timeout=1)
            if response.status_code == 200:
                data = response.json()
                angle = data.get("angle")
                distance = data.get("distance")
                self.info_text.insert(tk.END, f"Yol tespiti: Açısı = {angle:.2f}, Mesafe = {distance:.2f}\n")
                self.controller.turn_by_angle(angle)
                self.controller.move_distance(distance)
        except Exception as e:
            self.info_text.insert(tk.END, f"Path API hatası: {e}\n")
        self.root.after(1000, self.check_path_api)


if __name__ == "__main__":
    root = tk.Tk()
    controller = DroneController()  # Gerçek drone bağlantısı yapılır.
    app = DroneGUI(root, controller)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
