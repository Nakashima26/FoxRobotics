import time
import smbus
from mpu6050 import MPU6050
import adafruit_vl53l0x
import board
import busio

# Configurar el bus I2C
bus = smbus.SMBus(1)
mpu = MPU6050(bus)

# Inicializar VL53L0X
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_vl53l0x.VL53L0X(i2c)

# Función para leer el ángulo de Pitch y Roll del MPU6050
def get_mpu_angles():
    accel_data = mpu.get_accel_data()
    gyro_data = mpu.get_gyro_data()

    # Calcular los ángulos Pitch y Roll utilizando los datos del acelerómetro
    yaw = accel_data['z']  # Para simplificar, solo ejemplo básico

    return yaw

# Función principal
def main():
    while True:
        # Leer los ángulos del MPU6050
        pitch= get_mpu_angles()

        # Leer la distancia del VL53L0X
        dist = sensor.range
        dist -= 28

        # Imprimir los resultados
        print(f"MPU6050 -> Pitch: {pitch:.2f} | Roll: {roll:.2f}")
        print(f"VL53L0X -> Distancia: {distancia} mm")

        # Esperar un poco antes de la siguiente lectura
        time.sleep(0.2)

if __name__ == "__main__":
    main()