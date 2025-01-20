from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import cv2
import numpy as np
import time
import math
from rplidar import RPLidar

# Pixhawk bağlantısı
connection_string = "tcp:127.0.0.1:5762"  # Simülasyon bağlantısı için
print("Pixhawk'a bağlanılıyor...")
vehicle = connect(connection_string, wait_ready=True)

# Kamera ve LiDAR bağlantı ayarları
camera = cv2.VideoCapture(0)  # Kameranın bağlı olduğu port
# lidar = RPLidar("/dev/ttyUSB0")  # LiDAR cihazı bağlantı portu (Linux)


# Mesafe hesaplama fonksiyonu
def get_distance_metres(location1, location2):
    dlat = location2.lat - location1.lat
    dlon = location2.lon - location1.lon
    return math.sqrt((dlat**2) + (dlon**2)) * 1.113195e5


# Kalkış fonksiyonu
def arm_and_takeoff(target_altitude):
    print("Kalkışa hazırlanılıyor...")
    while not vehicle.is_armable:
        print("Araç hazır değil, bekleniyor...")
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Araç arm ediliyor...")
        time.sleep(1)
    print("Kalkış yapılıyor...")
    vehicle.simple_takeoff(target_altitude)
    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print(f"Şu anki irtifa: {altitude:.2f} metre")
        if altitude >= target_altitude * 0.95:
            print("Hedef irtifaya ulaşıldı.")
            break
        time.sleep(1)


# Kamera görüntü işleme fonksiyonu
def process_camera_feed():
    print("Kamera verisi işleniyor...")
    while True:
        ret, frame = camera.read()
        if not ret:
            print("Kamera akışı alınamıyor.")
            continue
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Belirli bir renk aralığını tespit et (örneğin, kırmızı hedef)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Konturların bulunması
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                print(f"Hedef algılandı: X={x}, Y={y}, Genişlik={w}, Yükseklik={h}")
                # Hedef yönlendirme (örnek, merkezde değilse sola hareket et)
                if x < frame.shape[1] // 3:
                    print("Hedef sol tarafta, sola yönleniliyor...")
                    send_ned_velocity(-1, 0, 0, 1)  # Yatay sola hareket
                elif x > 2 * frame.shape[1] // 3:
                    print("Hedef sağ tarafta, sağa yönleniliyor...")
                    send_ned_velocity(1, 0, 0, 1)  # Yatay sağa hareket
                else:
                    print("Hedefe ulaşıldı, duruluyor.")
                    return
        cv2.imshow("Kamera", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


# LiDAR ile engel algılama


# Yük bırakma
def drop_payload():
    print("Yük bırakma işlemi başlatılıyor...")
    pwm_channel = 9  # Servo bağlı olan Pixhawk kanalı
    pwm_open = 1900  # Servo açılma pozisyonu
    pwm_close = 1100  # Servo kapanma pozisyonu
    vehicle.channels.overrides[pwm_channel] = pwm_open
    time.sleep(2)
    vehicle.channels.overrides[pwm_channel] = pwm_close
    print("Yük bırakma tamamlandı.")


# Görev algoritması
def mission_execution():
    print("Görev başlatılıyor...")
    target_altitude = 10  # Hedef irtifa
    arm_and_takeoff(target_altitude)

    # İlk hedefe uçuş
    target_location = LocationGlobalRelative(37.7749, -122.4194, target_altitude)
    vehicle.simple_goto(target_location)

    # Kamera ile hedef tespiti
    process_camera_feed()

    # Engel algılama ve yük bırakma

    drop_payload()

    # İniş
    print("İniş yapılıyor...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.location.global_relative_frame.alt > 0.1:
        print(f"Şu anki irtifa: {vehicle.location.global_relative_frame.alt:.2f} metre")
        time.sleep(1)
    print("Görev tamamlandı!")


# Görev başlatma
try:
    mission_execution()
except KeyboardInterrupt:
    print("Görev durduruldu.")
finally:
    print("Bağlantı kesiliyor...")
    camera.release()
    cv2.destroyAllWindows()
    # lidar.stop()
    # lidar.disconnect()
    vehicle.close()
