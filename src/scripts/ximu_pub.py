#!/usr/bin/env python

import rospy
import ximureceiver, quaternion
import serial

from ximu.msg import EulerAngles
from ximu.msg import InertialMeasurements

def ximu_pub():
    pub_EulerAngles = rospy.Publisher('ximu_euler_angles', EulerAngles, queue_size=100)
    pub_IMU         = rospy.Publisher('ximu_inertial_measurements', InertialMeasurements, queue_size=100)
    rospy.init_node('ximu_pub')
    try:
        ximu = ximureceiver.XimuReceiver()
	s=serial.Serial("/dev/ttyUSB0", 115200)

    except:
        rospy.loginfo("ximu not connected")
        return
    while not rospy.is_shutdown():
        ximu.processNewChar(ord(s.read())%256)
               
        if ximu.isQuaternionGetReady():
            Quaternions                   = ximu.getQuaternion()
            Quaternion                    = quaternion.Quaternion(Quaternions.w, Quaternions.x, Quaternions.y, Quaternions.z)
            pac_EulerAngles               = Quaternion.getEulerAngles()
	    data_to_pub_EulerAngles       = EulerAngles()
            data_to_pub_EulerAngles.roll  = pac_EulerAngles.roll
	    data_to_pub_EulerAngles.pitch = pac_EulerAngles.pitch
	    data_to_pub_EulerAngles.yaw   = pac_EulerAngles.yaw
	    rospy.loginfo(data_to_pub_EulerAngles)
	    pub_EulerAngles.publish(data_to_pub_EulerAngles)

	if ximu.isInertialAndMagGetReady():
            InertialAndMag        = ximu.getInertialAndMag()
	    data_to_pub_IMU       = InertialMeasurements()
            data_to_pub_IMU.p     = InertialAndMag.gyrX
	    data_to_pub_IMU.q     = InertialAndMag.gyrY
	    data_to_pub_IMU.r     = InertialAndMag.gyrZ
            data_to_pub_IMU.a_x   = InertialAndMag.accX
	    data_to_pub_IMU.a_y   = InertialAndMag.accY
	    data_to_pub_IMU.a_z   = InertialAndMag.accZ
            data_to_pub_IMU.mag_x = InertialAndMag.magX
	    data_to_pub_IMU.mag_y = InertialAndMag.magY
	    data_to_pub_IMU.mag_z = InertialAndMag.magZ
	    pub_IMU.publish(data_to_pub_IMU)

if __name__ == '__main__':
    try:
        ximu_pub()
    except rospy.ROSInterruptException:
        pass
