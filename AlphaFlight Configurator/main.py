import serial
import time

ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS
)

commandTime = time.time() + 10

def recieve():
    s = ser.readline(2048)
    print(s)

if __name__ == '__main__':
    while 1:
        recieve()
        if time.time() > commandTime:
            ser.write(b'START_COMMUNICATION')
            recieve()
            time.sleep(10)
            ser.write(b'exit')
            commandTime = time.time() + 10.0
