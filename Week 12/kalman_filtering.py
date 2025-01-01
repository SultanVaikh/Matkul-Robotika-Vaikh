from controller import Robot

TIME_STEP = 32

# Fungsi Kalman Filter
def kalman_filter(z, u, x, P):
    x_pred = x + u
    P_pred = P + 0.1  # Noise proses
    K = P_pred / (P_pred + 1)  # Gain Kalman
    x = x_pred + K * (z - x_pred)  # Pembaruan posisi
    P = (1 - K) * P_pred  # Pembaruan ketidakpastian
    return x, P

# Inisialisasi robot
robot = Robot()

# Motor roda
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))  # Mode kecepatan
right_motor.setPosition(float('inf'))  # Mode kecepatan
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Sensor jarak
sensors = []
sensor_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    sensors.append(sensor)

# Variabel untuk Kalman Filter
x = 0.0  # Posisi awal
P = 1.0  # Ketidakpastian awal

# Threshold jarak untuk deteksi rintangan (sesuaikan nilai ini dengan eksperimen)
OBSTACLE_THRESHOLD = 100.0  # Nilai sensor jarak

# Loop utama
while robot.step(TIME_STEP) != -1:
    # Ambil nilai dari semua sensor jarak
    sensor_values = [sensor.getValue() for sensor in sensors]

    # Deteksi rintangan di sisi kiri, kanan, dan depan
    left_obstacle = sensor_values[5] > OBSTACLE_THRESHOLD or sensor_values[6] > OBSTACLE_THRESHOLD
    right_obstacle = sensor_values[1] > OBSTACLE_THRESHOLD or sensor_values[2] > OBSTACLE_THRESHOLD
    front_obstacle = sensor_values[0] > OBSTACLE_THRESHOLD or sensor_values[7] > OBSTACLE_THRESHOLD

    # Logika untuk menghindari rintangan
    if front_obstacle:
        # Jika ada rintangan di depan, mundur dan belok
        left_motor.setVelocity(-3.0)
        right_motor.setVelocity(3.0)
    elif left_obstacle:
        # Jika ada rintangan di kiri, belok ke kanan
        left_motor.setVelocity(3.0)
        right_motor.setVelocity(1.0)
    elif right_obstacle:
        # Jika ada rintangan di kanan, belok ke kiri
        left_motor.setVelocity(1.0)
        right_motor.setVelocity(3.0)
    else:
        # Jika tidak ada rintangan, jalan lurus
        left_motor.setVelocity(3.0)
        right_motor.setVelocity(3.0)

    # Ambil pengukuran sensor jarak terdekat (z) untuk Kalman Filter
    z = min(sensor_values)

    # Estimasi pergerakan robot (input u)
    u = 0.0  # Bisa diatur jika ada estimasi lain

    # Terapkan Kalman Filter
    x, P = kalman_filter(z, u, x, P)

    # Cetak estimasi posisi dan nilai sensor
    print(f"Estimasi Posisi Robot: {x}, Sensor Values: {sensor_values}")
