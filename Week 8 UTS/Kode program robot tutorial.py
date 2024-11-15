from controller import Robot

# Inisialisasi robot
robot = Robot()

# Waktu langkah simulasi (milidetik)
TIME_STEP = int(robot.getBasicTimeStep())

# Mendapatkan referensi motor kiri dan kanan
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# Mengatur motor ke mode velocity (kecepatan)
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Kecepatan roda untuk berputar
left_motor.setVelocity(2.0)   # Roda kiri bergerak maju
right_motor.setVelocity(-2.0)  # Roda kanan bergerak mundur

# Loop simulasi
while robot.step(TIME_STEP) != -1:
    pass  # Robot terus berputar di tempat
