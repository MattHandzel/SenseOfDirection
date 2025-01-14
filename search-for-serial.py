import serial
import time

ports = [
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
    "/dev/ttyACM0",
]

for port in ports:
    try:
        ser = serial.Serial(port, 9600, timeout=1)
        time.sleep(2)  # Give some time for data to arrive
        if ser.in_waiting > 0:
            print(f"Data available on {port}")
            print(ser.read(ser.in_waiting).decode("utf-8"))
        else:
            print(f"No data on {port}")
        ser.close()
    except serial.SerialException:
        print(f"Could not open {port}")
