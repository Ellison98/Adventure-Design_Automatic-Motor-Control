import busio

from board import SCL, SDA
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)

pca = PCA9685(i2c, address = 0x40)
pca.frequency = 62.5

angle = 90
offset = 0

angle += offset
angle = 180 if angle > 180 else (0 if angle < 0 else angle)

min_duty = int((750 * pca.frequency) / 1000000 * 0xFFFF)
max_duty = int((2250 * pca.frequency) / 1000000 * 0xFFFF)
duty_range = int(max_duty - min_duty)

duty_cycle = min_duty + int((angle / 180) * duty_range)

# pca.channels[15].duty_cycle = 5970
print(duty_cycle)
pca.channels[0].duty_cycle = duty_cycle