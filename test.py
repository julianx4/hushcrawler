import board
import busio
import adafruit_bno055
import time

i2c = busio.I2C(board.SCL, board.SDA)

# Create the sensor object using I2C.
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Read the Euler angles for heading, roll, pitch (all in degrees).
while True:
    euler = sensor.euler
    mag = sensor.magnetic
    gyro = sensor.gyro
    accel = sensor.acceleration
    linear_accel = sensor.linear_acceleration
    gravity = sensor.gravity