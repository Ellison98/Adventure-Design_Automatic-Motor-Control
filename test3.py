import busio

from board import SCL, SDA
from adafruit_pca9685 import PCA9685

import serial

import time

servo_pin = 0
throttle_pin = 15

arduino = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)

data_dict = {}

i2c = busio.I2C(SCL, SDA)

pca = PCA9685(i2c, address = 0x40)
pca.frequency = 62.5

angle = 90
offset = -2

speed = 6020
# d_speed = 5970
d_speed = 5950

st = 0

class servo_calc:
    def __init__(self):
        self.min_duty = int((750 * pca.frequency) / 1000000 * 0xFFFF)
        max_duty = int((2250 * pca.frequency) / 1000000 * 0xFFFF)
        self.duty_range = int(max_duty - self.min_duty)

    def __call__(self, angle):
        return self.min_duty + int((angle / 180) * self.duty_range)

servo_angle = servo_calc()

while True:
    try:
        data = arduino.readline()

        data = data.decode('ascii').split(' ')
        data = [for_a.split(':') for for_a in data]
        for for_a in data:
            data_dict[for_a[0]] = int(for_a[1].replace('\r\n', ''))

        # 200 UNDER = WHITE = 0
        # 2OO OVER = BLACK = 1

        # 010 -> FOWARD
        # 1X0 -> LEFT
        # 0X1 -> RIGHT
        # 000 -> DO

        # A2 -> RIGHT
        # A1 -> CENTER
        # A0 -> LEFT

        # 0 = LEFT
        # 90 = CENTER
        # 180 = RIGHT

        if data_dict['lr'] > 700 and data_dict['lr'] < 1200:
            angle = 0
        elif data_dict['lr'] > 1600:
            angle = 180
        else:
            # 000
            # Don't Care
            # 101
            # Don't Care
            # 001
            # RIGHT
            # 010
            # CENTER
            # 011
            # RIGHT
            # 100
            # LEFT
            # 110
            # LEFT
            # 111
            # STOP

            if data_dict['A0'] < 200 and data_dict['A1'] < 200 and data_dict['A2'] < 200:
                # 000":
                if angle < 90:
                    if angle < 75:
                        angle = 45
                    else:
                        angle -= 3
                else:
                    if angle > 105:
                        angle = 135
                    else:
                        angle += 3
            elif data_dict['A0'] > 200 and data_dict['A1'] < 200 and data_dict['A2'] > 200:
                # 101
                pass
            elif data_dict['A0'] < 200 and data_dict['A1'] > 200 and data_dict['A2'] < 200:
                # 010
                if angle < 90:
                    angle += 1
                elif angle > 90:
                    angle -= 1

                speed = d_speed
                st = 1
            elif data_dict['A0'] < 200 and data_dict['A1'] < 200 and data_dict['A2'] > 200:
                # 001
                angle = 120 if angle < 90 else angle
                angle += 1
            elif data_dict['A0'] < 200 and data_dict['A1'] > 200 and data_dict['A2'] > 200:
                # 011
                angle = 120 if angle < 90 else angle
                angle += 1
            elif data_dict['A0'] > 200 and data_dict['A1'] < 200 and data_dict['A2'] < 200:
                # 100
                angle = 60 if angle > 90 else angle
                angle -= 1
            elif data_dict['A0'] > 200 and data_dict['A1'] > 200 and data_dict['A2'] < 200:
                # 110
                angle = 60 if angle > 90 else angle
                angle -= 1
            elif data_dict['A0'] > 200 and data_dict['A1'] > 200 and data_dict['A2'] > 200:
                # 111
                if st == 1:
                    break

        angle = 180 if angle > 180 else (0 if angle < 0 else angle)

        calc_angle = angle + offset
        calc_angle = 180 if calc_angle > 180 else (0 if calc_angle < 0 else calc_angle)

        print(data_dict, angle, speed, calc_angle, st)

        pca.channels[0].duty_cycle = servo_angle(calc_angle)
        pca.channels[15].duty_cycle = speed
    except Exception as e:
        print(e)

calc_angle = 90 + offset
calc_angle = 180 if calc_angle > 180 else (0 if calc_angle < 0 else calc_angle)
pca.channels[0].duty_cycle = servo_angle(90)

time.sleep(1.5)

pca.channels[15].duty_cycle = 6020