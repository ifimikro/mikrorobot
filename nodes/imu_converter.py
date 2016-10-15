#!/usr/bin/env python

# Code partially ported from: https://github.com/sparkfun/MPU9150_Breakout

import rospy
import math

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion, Vector3

PI = 3.14159265359

# Global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
GyroMeasError = PI * (40.0 / 180.0) # Gyroscope measurement error in rads/s (shown as 3 deg/s)
GyroMeasDrift = PI * (0.0 / 180.0) # Gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)

# There is a tradeoff in the beta parameter between accuracy and response speed.
# In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
# However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
# Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
# By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
# I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
# the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
# In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
beta = math.sqrt(3.0 / 4.0) * GyroMeasError # Compute beta
zeta = math.sqrt(3.0 / 4.0) * GyroMeasDrift # Compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

# Global varible for the orientation quaternion
orientation = Quaternion(1, 0, 0, 0)

class PublisherAndSubscriber:

    imu = Imu()

    def _init_(self):
        rospy.init_node('imu_converter')
        print("Node initialized")
        self.pub = rospy.Publisher('imu', Imu, queue_size=10)
        self.sub = rospy.Subscriber('mpu', Float32MultiArray, self.fixMessage)
        self.lastReceivedTime = 0.0
        print("Created subscriber and publisher")

    def fixMessage(self, mpuRaw):

        self.deltaTime = rospy.Time.now().to_sec() - self.lastReceivedTime
        self.lastReceivedTime = rospy.Time.now().to_sec()

        # Magnetometer in milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
        # This needs calibration and is probably not very accurate
        cmpsX = mpuRaw.data[0]
        cmpsY = mpuRaw.data[1]
        cmpsZ = mpuRaw.data[2]

        # Linear acceleration in G's
        accelX = mpuRaw.data[3]
        accelY = mpuRaw.data[4]
        accelZ = mpuRaw.data[5]

        # Angular velocity in deg / s, converted to rad / s
        gyroX = mpuRaw.data[6] / 180.0 * PI
        gyroY = mpuRaw.data[7] / 180.0 * PI
        gyroZ = mpuRaw.data[8] / 180.0 * PI

        # Temperature in celsius. Used to calculate covariance for accelerometer
        # Currently ignored because it does not really matter (maximum sqrt(1.50)mG per Celsius deg)
        temperature = mpuRaw.data[9]

        # TODO: Covariance for gyroscope
        # Datasheet for MPU-9150: https://store.invensense.com/datasheets/invensense/MPU-9150_DataSheet_V4%203.pdf
        # Covariance for Gyroscope is according to datasheet sqrt(+-20 degrees) per second
        # From datasheet 6.1 Gyroscope Specifications - Zero-rate output

        # Calculate orientation quaternion
        self.MadgwickQuaternionUpdate(self.deltaTime, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, cmpsX, cmpsY, cmpsZ)

        #geometry_msgs/Quaternion
        self.imu.orientation = orientation
        #Float64[9] // Row major about x, y, z axes
        self.imu.orientation_covariance = (
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        )

        #geometry_msgs/Vector3
        # Converted to radians / sec
        self.imu.angular_velocity = Vector3(gyroX, gyroY, gyroZ)
        #Float64[9] // Row major about x, y, z axes
        self.imu.angular_velocity_covariance = (
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        )

        #geometry_msgs/Vector3
        self.imu.linear_acceleration = Vector3(accelX, accelY, accelZ)
        #Float64[9] // Row major x, y, z
        self.imu.linear_acceleration_covariance = (
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        )

        self.imu.header.stamp = rospy.Time.now()
        self.imu.header.frame_id = "imu_frame"
        self.pub.publish(self.imu)

    def MadgwickQuaternionUpdate(self, deltatime, ax, ay, az, gx, gy, gz, mx, my, mz):
        q1 = orientation.x
        q2 = orientation.y
        q3 = orientation.z
        q4 = orientation.w

        hx = hy = _2bx = _2bz = 0
        s1 = s2 = s3 = s4 = 0
        qDot1 = qDot2 = qDot3 = qDot4 = 0
        _2q1mx = _2q1my = _2q1mz = _2q1mw  = 0
        _4bx = _4bz = 0

        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _2q1q3 = 2.0 * q1 * q3
        _2q3q4 = 2.0 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        # Normalize accelerometer measurement
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0.0:
            return 0
        norm = 1 / norm
        ax *= norm
        ay *= norm
        az *= norm

        # Normalize magnetometer measurement
        norm = math.sqrt(mx * mx + my * my + mz * mz)
        if norm == 0.0:
            return 0
        norm = 1.0 / norm
        mx *= norm
        my *= norm
        mz *= norm

        # Reference direction of Earth's magnetic field
        _2q1mx = 2.0 * q1 * mx
        _2q1my = 2.0 * q1 * my
        _2q1mz = 2.0 * q1 * mz
        _2q2mx = 2.0 * q2 * mx
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz

        # Gradient decent algorithm corrective step
        s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        norm = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalise step magnitude
        norm = 1.0 / norm
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta * s4

        # Integrate to yield quaternion
        q1 += qDot1 * deltatime
        q2 += qDot2 * deltatime
        q3 += qDot3 * deltatime
        q4 += qDot4 * deltatime
        norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4) # Normalise quaternion
        norm = 1.0 / norm

        orientation.x = q1 * norm
        orientation.y = q2 * norm
        orientation.z = q3 * norm
        orientation.w = q4 * norm

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        PASObject = PublisherAndSubscriber()
        PASObject._init_()
        PASObject.run()
    except rospy.ROSInterruptException:
        exit()
