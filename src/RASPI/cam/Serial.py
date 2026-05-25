import serial
import time

# Abre el puerto serial para comunicarte con el Arduino
# Asegúrate de que el puerto sea correcto (/dev/ttyUSB0, /dev/ttyACM0, etc.)
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

time.sleep(2)  # Espera para que el puerto serial se establezca correctamente

# Enviar valor entre 0 y 640
valor = 350  # Este valor puede ser cualquier número dentro del rango 0-640

# Escribe el valor como una cadena seguida de un salto de línea
ser.write(f"{valor}\n".encode())

# Cierra la conexión serial después de enviar el valor
ser.close()
