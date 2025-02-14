from flask import Flask, jsonify
import random

app = Flask(__name__)


@app.route("/check_intersection")
def check_intersection():
    """
    Gerçek uygulamada bu endpoint, kameradan gelen görüntüyü işleyip
    4'lü kavşak tespiti yapar.
    Bu örnekte, %10 ihtimalle 4'lü kavşak tespit ediliyor.
    """
    four_way_found = random.random() < 0.2  # %10 ihtimal
    return jsonify({"four_way_intersection": four_way_found})


@app.route("/intersection_details")
def intersection_details():
    """
    Drone'un kavşaktaki konumuna bağlı olarak, drone'un orta konumda olup
    olmadığını ve dönüş için gereken açı ile ileri gidilecek mesafeyi belirler.

    Bu örnekte, dönüş açısı -45 ile 45 derece, mesafe ise 1 ile 5 metre arasında rastgele üretiliyor.
    """
    angle = random.uniform(-45, 45)
    distance = random.uniform(1, 5)
    return jsonify({"angle": angle, "distance": distance})


if __name__ == "__main__":
    # Uygulama tüm IP'lerden erişilebilir ve port 5000 üzerinden çalışır.
    app.run(host="0.0.0.0", port=5000)
