# PCA모듈 설정
# 서브 모터의 각도를 계산

import busio

from board import SCL, SDA
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)

pca = PCA9685(i2c, address = 0x40)
pca.frequency = 62.5

pca.channels[15].duty_cycle = 5970
# pca.channels[15].duty_cycle = 5840

pca.deinit()