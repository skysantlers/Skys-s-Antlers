import tkinter as tk
import cv2
from PIL import Image, ImageTk
import time
import random
import requests  # API istekleri için
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil


# ========================================================
# EK DONANIM SENSÖRLERİ SINIFI
# ========================================================
class AdditionalSensors:
    def __init__(self):
        pass

    def read_weight(self):
        return round(random.uniform(0, 1000), 2)  # gram cinsinden

    def read_temperature_and_humidity(self):
        temperature = round(random.uniform(20, 30), 2)  # °C
        humidity = round(random.uniform(30, 70), 2)  # %
        return temperature, humidity

    def read_pressure(self):
        return round(random.uniform(980, 1050), 2)  # hPa

    def read_acceleration(self):
        ax = round(random.uniform(-1, 1), 2)
        ay = round(random.uniform(-1, 1), 2)
        az = round(random.uniform(9.5, 10.5), 2)  # yerçekimi etkisi
        return ax, ay, az

    def read_lidar_distance(self):
        return round(random.uniform(0.2, 10.0), 2)  # metre cinsinden


# ========================================================
# DRONE KONTROL SINIFI (Simülasyon modu)
# ========================================================
class DroneController:
    def __init__(self, connection_string="tcp:127.0.0.1:5762"):
        self.connection_string = connection_string
        self.vehicle = None

    def connect_drone(self):
        print(f"Connecting to drone on: {self.connection_string}")
        self.vehicle = connect(self.connection_string, wait_ready=True)
        print("Connected to drone.")

    def arm_and_takeoff(self, target_altitude):
        print("Performing pre-arm checks...")
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle initialization...")
            time.sleep(1)
        print("Arming motors...")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)
        print("Taking off!")
        self.vehicle.simple_takeoff(target_altitude)
        while True:
            altitude = self.vehicle.location.global_relative_frame.alt
            print(f" Altitude: {altitude:.2f}")
            if altitude >= target_altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration=1):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111, 0, 0, 0,
            velocity_x, velocity_y, velocity_z,
            0, 0, 0, 0, 0
        )
        for _ in range(duration):
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            time.sleep(1)

    def move(self, direction):
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
            print("Simülasyon: İleri ve sola hareket")
            self.send_ned_velocity(1, -1, 0)
        elif direction == "up_right":
            print("Simülasyon: İleri ve sağa hareket")
            self.send_ned_velocity(1, 1, 0)
        elif direction == "down_left":
            print("Simülasyon: Geri ve sola hareket")
            self.send_ned_velocity(-1, -1, 0)
        elif direction == "down_right":
            print("Simülasyon: Geri ve sağa hareket")
            self.send_ned_velocity(-1, 1, 0)
        else:
            print("Simülasyon: Bilinmeyen hareket komutu:", direction)

    def turn_by_angle(self, angle):
        """
        Drone'un belirtilen açı kadar dönmesi için simülasyon.
        Gerçek uygulamada uygun komutlar (örneğin mavlink mesajları) gönderilecektir.
        """
        print(f"Simülasyon: Drone {angle:.2f} derece dönüyor.")
        time.sleep(1)

    def move_distance(self, distance):
        """
        Drone'un belirtilen mesafe kadar ileri gitmesini simüle eder.
        Hız 1 m/s kabul edilmiştir; mesafe kadar süre boyunca ileri hareket komutu gönderilir.
        """
        print(f"Simülasyon: Drone {distance:.2f} metre ilerliyor.")
        self.send_ned_velocity(1, 0, 0, duration=int(distance))

    def stop(self):
        self.send_ned_velocity(0, 0, 0)

    def land(self):
        self.vehicle.mode = VehicleMode("LAND")
        while self.vehicle.armed:
            time.sleep(1)
        print("Landed.")

    def disconnect(self):
        if self.vehicle:
            self.vehicle.close()
            print("Disconnected from drone.")


# ========================================================
# DRONE GUI, SÜRÜŞ ALGORİTMALARI VE TELEMETRİ
# ========================================================
class DroneGUI:
    def __init__(self, root, controller):
        self.root = root
        self.controller = controller
        self.root.title("IHA Kontrol Paneli")
        self.mode = None  # "autonomous" veya "manual"

        self.additional_sensors = AdditionalSensors()
        self.current_altitude = 5  # Varsayılan kalkış yüksekliği

        # --- Video Akışı Alanı ---
        self.video_label = tk.Label(root)
        self.video_label.grid(row=0, column=0, columnspan=4, padx=10, pady=10)

        # --- Üst Kontrol Butonları ---
        self.connect_button = tk.Button(
            root, text="Drone Bağlan", width=20, command=self.connect_drone
        )
        self.connect_button.grid(row=1, column=0, padx=5, pady=5)

        self.autonomous_button = tk.Button(
            root, text="Otonom Sürüş Başlat", width=20, command=self.start_autonomous
        )
        self.autonomous_button.grid(row=1, column=1, padx=5, pady=5)

        self.manual_button = tk.Button(
            root, text="Manuel Sürüş Başlat", width=20, command=self.start_manual
        )
        self.manual_button.grid(row=1, column=2, padx=5, pady=5)

        self.stop_button = tk.Button(root, text="Durdur", width=20, command=self.stop)
        self.stop_button.grid(row=1, column=3, padx=5, pady=5)

        # --- Alt Kontrol Butonları (İniş, Bağlantı Kes) ---
        self.land_button = tk.Button(root, text="İniş", width=20, command=self.land)
        self.land_button.grid(row=4, column=0, padx=5, pady=5)

        self.disconnect_button = tk.Button(
            root, text="Drone Bağlantısını Kes", width=20, command=self.disconnect_drone
        )
        self.disconnect_button.grid(row=4, column=1, padx=5, pady=5)

        # --- Manuel Kontrol: 8 Yönlü Butonlar + 2 Dikey Buton ---
        self.manual_frame = tk.Frame(root)
        self.manual_frame.grid(row=2, column=0, columnspan=4, padx=10, pady=10)

        # Yatay ve çapraz yönler
        self.btn_up_left = tk.Button(
            self.manual_frame,
            text="↖",
            width=5,
            command=lambda: self.manual_control("up_left"),
        )
        self.btn_up = tk.Button(
            self.manual_frame,
            text="↑",
            width=5,
            command=lambda: self.manual_control("up"),
        )
        self.btn_up_right = tk.Button(
            self.manual_frame,
            text="↗",
            width=5,
            command=lambda: self.manual_control("up_right"),
        )
        self.btn_left = tk.Button(
            self.manual_frame,
            text="←",
            width=5,
            command=lambda: self.manual_control("left"),
        )
        self.btn_right = tk.Button(
            self.manual_frame,
            text="→",
            width=5,
            command=lambda: self.manual_control("right"),
        )
        self.btn_down_left = tk.Button(
            self.manual_frame,
            text="↙",
            width=5,
            command=lambda: self.manual_control("down_left"),
        )
        self.btn_down = tk.Button(
            self.manual_frame,
            text="↓",
            width=5,
            command=lambda: self.manual_control("down"),
        )
        self.btn_down_right = tk.Button(
            self.manual_frame,
            text="↘",
            width=5,
            command=lambda: self.manual_control("down_right"),
        )
        # Dikey (3. boyut) yönler
        self.btn_upward = tk.Button(
            self.manual_frame,
            text="▲",
            width=5,
            command=lambda: self.manual_control("forward"),
        )
        self.btn_downward = tk.Button(
            self.manual_frame,
            text="▼",
            width=5,
            command=lambda: self.manual_control("backward"),
        )

        # Yerleşim düzeni:
        # İlk satır: Üç yatay yön
        self.btn_up_left.grid(row=0, column=0, padx=5, pady=5)
        self.btn_up.grid(row=0, column=1, padx=5, pady=5)
        self.btn_up_right.grid(row=0, column=2, padx=5, pady=5)
        # İkinci satır: Sol, dikey yukarı ve sağ
        self.btn_left.grid(row=1, column=0, padx=5, pady=5)
        self.btn_upward.grid(row=1, column=1, padx=5, pady=5)
        self.btn_right.grid(row=1, column=2, padx=5, pady=5)
        # Üçüncü satır: Çapraz aşağı yönler
        self.btn_down_left.grid(row=2, column=0, padx=5, pady=5)
        self.btn_down.grid(row=2, column=1, padx=5, pady=5)
        self.btn_down_right.grid(row=2, column=2, padx=5, pady=5)
        # Dördüncü satır: Dikey aşağı
        self.btn_downward.grid(row=3, column=1, padx=5, pady=5)

        self.hide_manual_controls()

        # --- Bilgi & Log Alanı ---
        self.info_text = tk.Text(root, height=10, width=80)
        self.info_text.grid(row=3, column=0, columnspan=4, padx=10, pady=10)
        self.info_text.insert(tk.END, "Sistem başlatıldı...\n")

        # --- Telemetri Alanı ---
        self.telemetry_label = tk.Label(
            root, text="Telemetri: Bekleniyor...", justify="left", font=("Courier", 10)
        )
        self.telemetry_label.grid(
            row=5, column=0, columnspan=4, padx=10, pady=10, sticky="w"
        )

        # --- Video Yakalama (Kamera) ---
        self.cap = cv2.VideoCapture(0)
        self.update_video()
        self.update_telemetry()
        # Not: API kontrolleri, otonom mod başlatıldığında çalıştırılacak.

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
            # Eğer otonom moddaysak, videodan alınan görüntü ile ek komutlar eklenebilir.
            if self.mode == "autonomous":
                self.controller.move("forward")
        self.root.after(100, self.update_video)

    def update_telemetry(self):
        telemetry_str = ""
        telemetry_str += (
            f"Control Mode: {self.mode if self.mode is not None else 'None'}\n"
        )
        weight = self.additional_sensors.read_weight()
        temp, humidity = self.additional_sensors.read_temperature_and_humidity()
        pressure = self.additional_sensors.read_pressure()
        accel = self.additional_sensors.read_acceleration()
        lidar = self.additional_sensors.read_lidar_distance()

        telemetry_str += f"Weight: {weight} g\n"
        telemetry_str += f"Temperature: {temp} °C, Humidity: {humidity} %\n"
        telemetry_str += f"Pressure: {pressure} hPa\n"
        telemetry_str += f"Acceleration: ax: {accel[0]} m/s², ay: {accel[1]} m/s², az: {accel[2]} m/s²\n"
        telemetry_str += f"Lidar Distance: {lidar} m\n"
        telemetry_str += f"Altitude: {self.current_altitude:.2f} m\n"

        self.telemetry_label.config(text=telemetry_str)
        self.root.after(1000, self.update_telemetry)

    def start_autonomous(self):
        self.mode = "autonomous"
        self.hide_manual_controls()
        self.info_text.insert(tk.END, "Otonom sürüş başlatıldı...\n")
        try:
            self.controller.arm_and_takeoff(5)
            # Otonom mod başlatıldığında API kontrolleri de çalışmaya başlasın.
            self.check_intersection_api()
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
        """
        Otonom moddayken, her saniye backend'e "/check_intersection" isteği gönderilir.
        Eğer dörtlü kavşağın ortası tespit edilirse (intersection), drone durdurulur ve
        ardından "/intersection_details" isteği ile o orta konuma göre hesaplanan
        dönüş açısı ve mesafe alınır. Bu değerlerle drone yönlendirilir.
        """
        if self.mode != "autonomous":
            self.root.after(1000, self.check_intersection_api)
            return

        try:
            response = requests.get("http://localhost:5000/check_intersection", timeout=0.5)
            if response.status_code == 200:
                data = response.json()
                if data.get("four_way_intersection", False):
                    self.info_text.insert(tk.END, "4'lü kavşak tespit edildi! Drone durduruluyor...\n")
                    self.controller.stop()
                    try:
                        details_response = requests.get("http://localhost:5000/intersection_details", timeout=0.5)
                        if details_response.status_code == 200:
                            details = details_response.json()
                            angle = details.get("angle")
                            distance = details.get("distance")
                            self.info_text.insert(
                                tk.END,
                                f"Kavşak merkezi detayları: Açısı = {angle:.2f} derece, Mesafe = {distance:.2f} metre\n",
                            )
                            self.controller.turn_by_angle(angle)
                            self.controller.move_distance(distance)
                        else:
                            self.info_text.insert(tk.END, "Intersection details API yanıtı hatalı.\n")
                    except Exception as e:
                        self.info_text.insert(tk.END, f"Intersection details API hatası: {e}\n")
        except Exception as e:
            self.info_text.insert(tk.END, f"Intersection API hatası: {e}\n")

        self.root.after(1000, self.check_intersection_api)


if __name__ == "__main__":
    root = tk.Tk()
    controller = DroneController()
    app = DroneGUI(root, controller)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()