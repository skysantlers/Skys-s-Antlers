#!/usr/bin/env python3
import tkinter as tk
import cv2
from PIL import Image, ImageTk
import time
import requests
import random

# Gerçek drone kontrolü için DroneKit kullanımı
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil


# --------------------------------------------------------
# Simülasyon Sensör Entegrasyonu (AdditionalSensors)
# --------------------------------------------------------
class AdditionalSensors:
    def __init__(self):
        # Simülasyon için sabit veya rastgele değerler üretilecek.
        pass

    def read_weight(self):
        # Örneğin 100-500 gram arasında rastgele bir değer.
        return round(random.uniform(100, 500), 2)

    def read_temperature_and_humidity(self):
        # Sıcaklık 20-30 °C, nem 40-60 % arasında rastgele değerler
        temperature = round(random.uniform(20, 30), 2)
        humidity = round(random.uniform(40, 60), 2)
        return temperature, humidity

    def read_pressure(self):
        # Örneğin 990-1020 hPa arasında rastgele bir değer.
        return round(random.uniform(990, 1020), 2)

    def read_acceleration(self):
        # Basitçe x,y için -1 ile 1 arasında ve z için 9.5-10.5 (yerçekimi)
        ax = round(random.uniform(-1, 1), 2)
        ay = round(random.uniform(-1, 1), 2)
        az = round(random.uniform(9.5, 10.5), 2)
        return ax, ay, az

    def read_lidar_distance(self):
        # Örneğin 0.5-10 metre arasında rastgele mesafe
        return round(random.uniform(0.5, 10), 2)


# --------------------------------------------------------
# Gerçek Drone Kontrolü (DroneController) - Simülasyon modunda çalışıyor
# --------------------------------------------------------
class DroneController:
    def __init__(self, connection_string="127.0.0.1:14550", baud=57600):
        print("Simülasyon modu: Gerçek drone bağlantısı kurulmayacak.")
        # Simülasyon modunda gerçek bağlantı yapılmayacak, sadece mesajlar gösterilecek.
        self.vehicle = None

    def connect_drone(self):
        print("Simülasyon: Drone bağlantısı başarılı.")

    def arm_and_takeoff(self, target_altitude):
        print("Simülasyon: Drone arm ediliyor ve kalkış yapılıyor...")
        time.sleep(2)
        print(f"Simülasyon: {target_altitude} metre yüksekliğe ulaşıldı.")

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration=1):
        print(f"Simülasyon: vx={velocity_x}, vy={velocity_y}, vz={velocity_z} ile {duration} sn hareket.")
        time.sleep(duration)

    def move(self, direction):
        # Manuel kontrol için diagonal (ara) yönler de eklenmiştir.
        if direction == "forward":
            print("Simülasyon: İleri hareket")
            self.send_ned_velocity(1, 0, 0)
        elif direction == "backward":
            print("Simülasyon: Geri hareket")
            self.send_ned_velocity(-1, 0, 0)
        elif direction == "left":
            print("Simülasyon: Sola hareket")
            self.send_ned_velocity(0, -1, 0)
        elif direction == "right":
            print("Simülasyon: Sağa hareket")
            self.send_ned_velocity(0, 1, 0)
        elif direction == "up":
            print("Simülasyon: Yukarı hareket")
            self.send_ned_velocity(0, 0, -1)
        elif direction == "down":
            print("Simülasyon: Aşağı hareket")
            self.send_ned_velocity(0, 0, 1)
        elif direction == "up_left":
            print("Simülasyon: Sol üst hareket")
            self.send_ned_velocity(1, -1, 0)
        elif direction == "up_right":
            print("Simülasyon: Sağ üst hareket")
            self.send_ned_velocity(1, 1, 0)
        elif direction == "down_left":
            print("Simülasyon: Sol alt hareket")
            self.send_ned_velocity(-1, -1, 0)
        elif direction == "down_right":
            print("Simülasyon: Sağ alt hareket")
            self.send_ned_velocity(-1, 1, 0)
        else:
            print("Simülasyon: Bilinmeyen hareket komutu:", direction)

    def turn_by_angle(self, angle):
        print(f"Simülasyon: {angle:.2f} derece dönüyor.")
        time.sleep(2)

    def move_distance(self, distance):
        print(f"Simülasyon: {distance:.2f} metre ilerliyor.")
        self.send_ned_velocity(1, 0, 0, duration=distance)

    def stop(self):
        print("Simülasyon: Drone durdu.")
        self.send_ned_velocity(0, 0, 0)

    def land(self):
        print("Simülasyon: Drone inişe geçti.")
        time.sleep(2)
        print("Simülasyon: Drone indi.")

    def disconnect(self):
        print("Simülasyon: Drone bağlantısı kesildi.")


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
        # 3x3 ızgara yerleşimi (merkez hücre boş bırakıldı)
        self.btn_up_left.grid(row=0, column=0, padx=5, pady=5)
        self.btn_up.grid(row=0, column=1, padx=5, pady=5)
        self.btn_up_right.grid(row=0, column=2, padx=5, pady=5)
        self.btn_left.grid(row=1, column=0, padx=5, pady=5)
        # Merkez boş
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
    controller = DroneController()  # Simülasyon modunda çalışacak
    app = DroneGUI(root, controller)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
