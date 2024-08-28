import serial
import logging
import time
from threading import Thread

# Thiết lập logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def read_tfluna_data(ser, sensor_name):
    while True:
        counter = ser.in_waiting
        if counter > 8:
            bytes_serial = ser.read(9)
            ser.reset_input_buffer()

            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                distance = bytes_serial[2] + bytes_serial[3] * 256
                strength = bytes_serial[4] + bytes_serial[5] * 256
                temperature = bytes_serial[6] + bytes_serial[7] * 256
                temperature = (temperature / 8.0) - 256.0
                #logger.info('%s - Distance: %.2f cm, Strength: %.0f / 65535 (16-bit), Chip Temperature: %.1f C', sensor_name, distance, strength, temperature)
                logger.info('%s - Distance: %.2f cm', sensor_name, distance)
                

def main():
    ser1 = serial.Serial("/dev/ttyUSB0", 115200, timeout=0)
    ser2 = serial.Serial("/dev/ttyUSB1", 115200, timeout=0)

    try:
        if not ser1.isOpen():
            ser1.open()
        if not ser2.isOpen():
            ser2.open()

        thread1 = Thread(target=read_tfluna_data, args=(ser1, "Sensor 1"))
        thread2 = Thread(target=read_tfluna_data, args=(ser2, "Sensor 2"))

        thread1.start()
        thread2.start()

        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        ser1.close()
        ser2.close()

if __name__ == "__main__":
    main()