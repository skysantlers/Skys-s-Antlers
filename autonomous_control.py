import tkinter as tk
import cv2
from PIL import Image, ImageTk
import time
import random
import math


# ========================================================
# EK DONANIM SENSÖRLERİ SINIFI
# (Gerçek uygulamada ilgili kütüphaneleri import edip sensörleri
#  initialize etmeniz gerekmektedir.)
# ========================================================
class AdditionalSensors:
    def __init__(self):
        # Gerçek sensörlerde: GPIO, I2C, UART, vb. bağlantılar burada başlatılır.
        # Örneğin:
        #   import Adafruit_DHT
        #   self.dht_sensor = Adafruit_DHT.DHT22
        #   self.dht_pin = 4
        #
        #   import smbus
        #   self.bus = smbus.SMBus(1)
        #   self.bmp280_address = 0x76
        #
        #   # HX711 için özel kütüphane vb.
        pass

    def read_weight(self):
        # HX711 yük hücresi ile gerçek ölçüm yapacaksanız
        # ilgili kütüphaneyi kullanarak değeri okuyun.
        # Şimdilik simülasyon:
        return round(random.uniform(0, 1000), 2)  # gram cinsinden

    def read_temperature_and_humidity(self):
        # DHT22 ile gerçek ölçüm yapılacaksa, Adafruit_DHT kütüphanesini kullanın.
        # Şimdilik simülasyon:
        temperature = round(random.uniform(20, 30), 2)  # °C
        humidity = round(random.uniform(30, 70), 2)  # %
        return temperature, humidity

    def read_pressure(self):
        # BMP280 ile gerçek ölçüm yapılacaksa, ilgili kütüphaneyi kullanın.
        # Şimdilik simülasyon:
        return round(random.uniform(980, 1050), 2)  # hPa

    def read_acceleration(self):
        # MPU6050 ivme sensörü için gerçek veriyi okuyun.
        # Şimdilik simülasyon (x, y, z m/s²):
        ax = round(random.uniform(-1, 1), 2)
        ay = round(random.uniform(-1, 1), 2)
        az = round(random.uniform(9.5, 10.5), 2)  # yerçekimi etkisi
        return ax, ay, az

    def read_lidar_distance(self):
        # TFmini Plus Lidar modülü için seri port üzerinden veri okunabilir.
        # Şimdilik simülasyon (metre cinsinden):
        return round(random.uniform(0.2, 10.0), 2)


# ========================================================
# DRONE KONTROL SINIFI (DroneKit veya Simülasyon)
# ========================================================
try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative
    from pymavlink import mavutil
except ImportError:
    print("DroneKit veya pymavlink modülleri bulunamadı. Simülasyon modu aktif olacak.")

    class DroneController:
        def __init__(self, connection_string="127.0.0.1:14550"):
            print("Simülasyon modu: Gerçek drone bağlantısı kurulmayacak.")
            self.vehicle = None

        def connect_drone(self):
            print("Simülasyon: Drone bağlantısı başarılı.")

        def arm_and_takeoff(self, target_altitude):
            print(f"Simülasyon: {target_altitude} metre yüksekliğe kalkış yapılıyor.")
            time.sleep(2)

        def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration=1):
            print(
                f"Simülasyon: {velocity_x}, {velocity_y}, {velocity_z} hızlarıyla {duration} sn hareket."
            )
            time.sleep(duration)

        def move(self, direction):
            if direction == "forward":
                print("Moving forward")
                self.send_ned_velocity(1, 0, 0)
            elif direction == "backward":
                print("Moving backward")
                self.send_ned_velocity(-1, 0, 0)
            elif direction == "left":
                print("Moving left")
                self.send_ned_velocity(0, -1, 0)
            elif direction == "right":
                print("Moving right")
                self.send_ned_velocity(0, 1, 0)
            elif direction == "up":
                print("Moving up")
                self.send_ned_velocity(0, 0, -1)
            elif direction == "down":
                print("Moving down")
                self.send_ned_velocity(0, 0, 1)
            elif direction in ["center", "hover"]:
                print("Hovering")
                self.send_ned_velocity(0, 0, 0)
            elif direction == "up_left":
                print("Moving up left")
                self.send_ned_velocity(0.7, -0.7, 0)
            elif direction == "up_right":
                print("Moving up right")
                self.send_ned_velocity(0.7, 0.7, 0)
            elif direction == "down_left":
                print("Moving down left")
                self.send_ned_velocity(-0.7, -0.7, 0)
            elif direction == "down_right":
                print("Moving down right")
                self.send_ned_velocity(-0.7, 0.7, 0)
            else:
                print("Unknown direction:", direction)

        def stop(self):
            print("Simülasyon: Drone durdu (hover modunda).")
            self.send_ned_velocity(0, 0, 0)

        def land(self):
            print("Simülasyon: Drone inişe geçti.")
            time.sleep(2)
            print("Simülasyon: Drone indi.")

        def disconnect(self):
            print("Simülasyon: Drone bağlantısı kesildi.")

else:

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
            # Basit bekleme; gerçek uygulamada daha sağlam bir algoritma kullanılmalı
            while True:
                altitude = self.vehicle.location.global_relative_frame.alt
                print(f" Altitude: {altitude:.2f}")
                if altitude >= target_altitude * 0.95:
                    print("Reached target altitude")
                    break
                time.sleep(1)

        def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration=1):
            if self.vehicle is None:
                # Simülasyon modu: Gerçek drone bağlantısı yok
                print(
                    f"Simulated velocity command: vx={velocity_x}, vy={velocity_y}, vz={velocity_z} for {duration} seconds"
                )
                time.sleep(duration)
                return

            # Gerçek drone için komut gönderme:
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,  # time_boot_ms (kullanılmıyor)
                0,
                0,  # target system, target component
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # coordinate frame
                0b0000111111000111,  # type_mask: sadece hız bileşenleri aktif
                0,
                0,
                0,  # x, y, z konumları (kullanılmıyor)
                velocity_x,
                velocity_y,
                velocity_z,  # vx, vy, vz (m/s)
                0,
                0,
                0,  # ivme (kullanılmıyor)
                0,
                0,  # yaw, yaw_rate
            )
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            time.sleep(duration)

        def move(self, direction):
            if direction == "forward":
                print("Moving forward")
                self.send_ned_velocity(1, 0, 0)
            elif direction == "backward":
                print("Moving backward")
                self.send_ned_velocity(-1, 0, 0)
            elif direction == "left":
                print("Moving left")
                self.send_ned_velocity(0, -1, 0)
            elif direction == "right":
                print("Moving right")
                self.send_ned_velocity(0, 1, 0)
            elif direction == "up":
                print("Moving up")
                self.send_ned_velocity(0, 0, -1)
            elif direction == "down":
                print("Moving down")
                self.send_ned_velocity(0, 0, 1)
            elif direction in ["center", "hover"]:
                print("Hovering")
                self.send_ned_velocity(0, 0, 0)
            elif direction == "up_left":
                print("Moving up left")
                self.send_ned_velocity(0.7, -0.7, 0)
            elif direction == "up_right":
                print("Moving up right")
                self.send_ned_velocity(0.7, 0.7, 0)
            elif direction == "down_left":
                print("Moving down left")
                self.send_ned_velocity(-0.7, -0.7, 0)
            elif direction == "down_right":
                print("Moving down right")
                self.send_ned_velocity(-0.7, 0.7, 0)
            else:
                print("Unknown direction:", direction)

        def stop(self):
            print("Stopping, hovering")
            self.send_ned_velocity(0, 0, 0)

        def land(self):
            print("Landing...")
            self.vehicle.mode = VehicleMode("LAND")
            time.sleep(5)
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

        # EK SENSÖR NESNESİ: Aşağıdaki nesne, diğer sensörlerin okunması için kullanılacak.
        self.additional_sensors = AdditionalSensors()

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

        # --- Manuel Kontrol: 8 Yönlü Butonlar ---
        self.manual_frame = tk.Frame(root)
        self.manual_frame.grid(row=2, column=0, columnspan=4, padx=10, pady=10)

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

        self.btn_up_left.grid(row=0, column=0, padx=5, pady=5)
        self.btn_up.grid(row=0, column=1, padx=5, pady=5)
        self.btn_up_right.grid(row=0, column=2, padx=5, pady=5)
        self.btn_left.grid(row=1, column=0, padx=5, pady=5)
        self.btn_right.grid(row=1, column=2, padx=5, pady=5)
        self.btn_down_left.grid(row=2, column=0, padx=5, pady=5)
        self.btn_down.grid(row=2, column=1, padx=5, pady=5)
        self.btn_down_right.grid(row=2, column=2, padx=5, pady=5)

        self.hide_manual_controls()

        # --- Bilgi & Log Alanı ---
        self.info_text = tk.Text(root, height=10, width=80)
        self.info_text.grid(row=3, column=0, columnspan=4, padx=10, pady=10)
        self.info_text.insert(tk.END, "Sistem başlatıldı...\n")

        # --- Telemetri Bilgilerinin Görüntüleneceği Alan ---
        self.telemetry_label = tk.Label(
            root, text="Telemetri: Bekleniyor...", justify="left", font=("Courier", 10)
        )
        self.telemetry_label.grid(
            row=5, column=0, columnspan=4, padx=10, pady=10, sticky="w"
        )

        # --- Video Yakalama (Kamera) ---
        self.cap = cv2.VideoCapture(0)  # Raspberry Pi Kamera modülü veya USB kamera
        self.update_video()
        self.update_telemetry()  # Telemetri güncelleme döngüsünü başlat

        # --- Otonom Kontrol: Hedef Konum Kutucukları ---
        self.goto_frame = tk.Frame(root)
        self.goto_frame.grid(row=2, column=0, columnspan=4, padx=10, pady=10)
        self.goto_frame.grid_remove()

        self.lat_label = tk.Label(self.goto_frame, text="Latitude:")
        self.lat_label.grid(row=0, column=0, padx=5, pady=5)
        self.lat_entry = tk.Entry(self.goto_frame, width=10)
        self.lat_entry.grid(row=0, column=1, padx=5, pady=5)
        self.lon_label = tk.Label(self.goto_frame, text="Longitude:")
        self.lon_label.grid(row=0, column=2, padx=5, pady=5)
        self.lon_entry = tk.Entry(self.goto_frame, width=10)
        self.lon_entry.grid(row=0, column=3, padx=5, pady=5)
        self.alt_label = tk.Label(self.goto_frame, text="Altitude:")
        self.alt_label.grid(row=0, column=4, padx=5, pady=5)
        self.alt_entry = tk.Entry(self.goto_frame, width=10)
        self.alt_entry.grid(row=0, column=5, padx=5, pady=5)

        self.goto_button = tk.Button(
            self.goto_frame, text="Hedefe Git", width=20, command=lambda: self.goto_waypoint(
                float(self.lat_entry.get()), float(self.lon_entry.get()), float(self.alt_entry.get())
            )
        )
        self.goto_button.grid(row=0, column=6, padx=5, pady=5)


    # --- Drone Bağlantısı ve Kontrol Fonksiyonları ---
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

    def hide_goto_controls(self):
        self.goto_frame.grid_remove()

    def show_goto_controls(self):
        self.goto_frame.grid()

    def show_manual_controls(self):
        self.manual_frame.grid()

    def manual_control(self, direction):
        self.info_text.insert(tk.END, f"Manuel kontrol: {direction}\n")
        self.controller.move(direction)

    def autonomous_decision(self, frame):
        """
        Gerçek uygulamada bu metotta YOLOv10 veya benzeri bir model kullanılarak
        yol, trafik ışığı, araç ve insan tespiti yapılır; ardından karar verilir.
        Aşağıdaki örnekte, %5 ihtimalle trafik ışığı tespit ediliyormuş gibi davranılır.
        """
        if random.random() < 0.05:
            self.info_text.insert(
                tk.END, "Trafik ışığı tespit edildi, merkeze yönlendiriliyor...\n"
            )
            return "center"
        else:
            return "forward"

    def update_video(self):
        ret, frame = self.cap.read()
        if ret:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=image)
            self.video_label.imgtk = imgtk
            self.video_label.configure(image=imgtk)
            if self.mode == "autonomous":
                decision = self.autonomous_decision(frame)
                self.controller.move(decision)
        self.root.after(100, self.update_video)

    def update_telemetry(self):
        """
        Drone'dan (veya simülasyon modunda) ve ek sensörlerden alınan tüm verileri
        (kontrol modu, altitude, GPS, battery, araç modu, konum, ağırlık, sıcaklık, nem,
         basınç, ivme, Lidar mesafesi vb.) güncelleyip GUI üzerinde gösterir.
        """
        telemetry_str = ""
        telemetry_str += (
            f"Control Mode: {self.mode if self.mode is not None else 'None'}\n"
        )
        if self.controller.vehicle is not None:
            try:
                alt = self.controller.vehicle.location.global_relative_frame.alt
                telemetry_str += f"Altitude: {alt:.2f} m\n"
            except Exception:
                telemetry_str += "Altitude: N/A\n"
            try:
                gps = self.controller.vehicle.gps_0
                telemetry_str += f"GPS: Fix Type: {gps.fix_type}, Satellites: {gps.satellites_visible}\n"
            except Exception:
                telemetry_str += "GPS: N/A\n"
            try:
                battery = self.controller.vehicle.battery
                level = battery.level if battery.level is not None else "N/A"
                telemetry_str += f"Battery: {level}%\n"
            except Exception:
                telemetry_str += "Battery: N/A\n"
            try:
                mode = self.controller.vehicle.mode.name
                telemetry_str += f"Vehicle Mode: {mode}\n"
            except Exception:
                telemetry_str += "Vehicle Mode: N/A\n"
            try:
                lat = self.controller.vehicle.location.global_frame.lat
                lon = self.controller.vehicle.location.global_frame.lon
                telemetry_str += f"Location: lat: {lat}, lon: {lon}\n"
            except Exception:
                telemetry_str += "Location: N/A\n"
        else:
            telemetry_str += "Simülasyon modu: Drone telemetri verisi yok.\n"

        # EK SENSÖR VERİLERİ (Simülasyon)
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

        self.telemetry_label.config(text=telemetry_str)
        self.root.after(1000, self.update_telemetry)  # Her 1 saniyede güncelle

    def goto_waypoint(self, latitude, longitude, altitude):
        """
        Drone'u belirtilen GPS koordinatlarına götürür.
        """
        self.info_text.insert(tk.END, f"Konumuna gidiliyor: Lat={latitude}, Lon={longitude}, Alt={altitude}\n")
        print(f"Going to waypoint: Lat={latitude}, Lon={longitude}, Alt={altitude}\n")
        target_location = LocationGlobalRelative(latitude, longitude, altitude)
        self.controller.vehicle.simple_goto(target_location)
        # Hedefe ulaşılana kadar bekle
        while True:
            current_location = self.controller.vehicle.location.global_frame
            distance = self.get_distance_metres(current_location, target_location)
            print(f"Distance to waypoint: {distance:.2f} meters")
            if distance <= 1.0:  # 1 metre yakınlıkta kabul et
                print("Reached waypoint")
                self.info_text.insert(tk.END, "Hedef noktasına ulaşıldı.\n")
                break
            time.sleep(1)

    def get_distance_metres(self, location1, location2):
        """
        İki GPS konumu arasındaki mesafeyi hesaplar (metre cinsinden).
        """
        dlat = location2.lat - location1.lat
        dlong = location2.lon - location1.lon
        return ((dlat ** 2 + dlong ** 2) ** 0.5) * 1.113195e5

    def start_autonomous(self):
        self.mode = "autonomous"
        self.show_goto_controls()
        self.hide_manual_controls()
        self.info_text.insert(tk.END, "Otonom sürüş başlatıldı...\n") # -35.359996 149.167126 15 Örnek koordinatlar
        try:
            self.controller.arm_and_takeoff(5)  # 5 metre yüksekliğe kalkış
        except Exception as e:
            self.info_text.insert(tk.END, f"Kalkış sırasında hata: {e}\n")

    def start_manual(self):
        self.mode = "manual"
        self.hide_goto_controls()
        self.show_manual_controls()
        self.info_text.insert(tk.END, "Manuel sürüş başlatıldı...\n")
        try:
            self.controller.arm_and_takeoff(5)
        except Exception as e:
            self.info_text.insert(tk.END, f"Kalkış sırasında hata: {e}\n")

    def stop(self):
        self.mode = None
        self.hide_manual_controls()
        self.hide_goto_controls()
        self.info_text.insert(tk.END, "Sistem durduruldu.\n")
        self.controller.stop()

    def land(self):
        self.mode = None
        self.hide_manual_controls()
        self.hide_goto_controls()
        self.info_text.insert(tk.END, "İniş komutu gönderildi.\n")
        self.controller.land()

    def on_closing(self):
        self.cap.release()
        self.controller.disconnect()
        self.root.destroy()


# ========================================================
# Main: Uygulamayı Başlat
# ========================================================
if __name__ == "__main__":
    root = tk.Tk()
    controller = (
        DroneController()
    )  # Gerekirse bağlantı adresini düzenleyin (örn. SITL, gerçek drone)
    app = DroneGUI(root, controller)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()