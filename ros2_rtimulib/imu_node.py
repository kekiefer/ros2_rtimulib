
import os
import math

import RTIMU

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

from ros2_rtimulib.imu_utils import get_altitude, G_TO_MPSS
from ros2_rtimulib import better_get_parameter_or


class IMUNode(Node):
    """Uses the RTIMULib to grab sensor data and publish it
    
    Ref:
        * https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg
        * inspired by: https://github.com/romainreignier/rtimulib_ros/
        * https://github.com/RPi-Distro/RTIMULib
        * https://github.com/RTIMULib/RTIMULib2
        * https://github.com/mattanimation/RTIMULib2/tree/master/Linux/python

    """

    def __init__(self):
        super().__init__('rtimulib_driver', namespace="", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.frame_id = better_get_parameter_or(self, 'frame_id', 'imu').value
        # should be a .ini file but we don't include the extension
        settings_file_root = get_package_share_directory('ros2_rtimulib')
        self.settings_filename = better_get_parameter_or(self, 'settings_file', "RTIMULib").value
        self.settings_file = os.path.join(settings_file_root, self.settings_filename)

        self.get_logger().info("Using settings file " + self.settings_file)
        if not os.path.exists(self.settings_file):
            self.get_logger().info("Settings file does not exist, will be created")

        s = RTIMU.Settings(self.settings_file)
        self.imu = RTIMU.RTIMU(s)
        self.pressure = RTIMU.RTPressure(s)

        self.get_logger().info("IMU Name: " + self.imu.IMUName())
        self.get_logger().info("Pressure Name: " + self.pressure.pressureName())

        if not self.imu.IMUInit():
            self.get_logger().info("IMU Init Failed")
            self.context.shutdown()
            return
        else:
            self.get_logger().info("IMU Init Succeeded")

        # this is a good time to set any fusion parameters
        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)

        if (not self.pressure.pressureInit()):
            self.get_logger().info("Pressure sensor Init Failed")
        else:
            self.get_logger().info("Pressure sensor Init Succeeded")

        poll_interval = self.imu.IMUGetPollInterval()
        self.get_logger().info("Recommended Poll Interval: {0}mS\n".format(poll_interval))

        
        # create pubs
        self.imu_pub = self.create_publisher(Imu, '/imu', qos_profile=10)

        # create a rate with fixed hz rate to pub at
        # hz_rate = (poll_interval*1.0/10.0)
        # self.get_logger().info('hz rate is: {0}'.format(hz_rate))
        # imu_rate = self.create_rate(hz_rate)

        timer_rate = (poll_interval * 1.0 / 1000.0)
        self.get_logger().info("timer_rate: {0}".format(timer_rate))
        self.pub_tmr = self.create_timer(timer_rate, self.tmr_cb)

        # if using rate, which doesn't seem to work?
        #while self.context.ok():
        #
        #    self.handle_imu()
        #
        #    imu_rate.sleep()

    def tmr_cb(self):
        self.handle_imu()

    def handle_imu(self):
        if self.imu.IMURead():
            # x, y, z = imu.getFusionData()
            # print("%f %f %f" % (x,y,z))
            data = self.imu.getIMUData()
            """
            data keys:
                "timestamp", data.timestamp,
                "fusionPoseValid", PyBool_FromLong(data.fusionPoseValid),
                "fusionPose", data.fusionPose.x(), data.fusionPose.y(), data.fusionPose.z(),
                "fusionQPoseValid", PyBool_FromLong(data.fusionQPoseValid),
                "fusionQPose", data.fusionQPose.scalar(), data.fusionQPose.x(), data.fusionQPose.y(), data.fusionQPose.z(),
                "gyroValid", PyBool_FromLong(data.gyroValid),
                "gyro", data.gyro.x(), data.gyro.y(), data.gyro.z(),
                "accelValid", PyBool_FromLong(data.accelValid),
                "accel", data.accel.x(), data.accel.y(), data.accel.z(),
                "compassValid", PyBool_FromLong(data.compassValid),
                "compass", data.compass.x(), data.compass.y(), data.compass.z(),
                "pressureValid", PyBool_FromLong(data.pressureValid),
                "pressure", data.pressure,
                "temperatureValid", PyBool_FromLong(data.temperatureValid),
                "temperature", data.temperature,
                "humidityValid", PyBool_FromLong(data.humidityValid),
                "humidity", data.humidity);
            """

            if data is not None:
                # self.get_logger().info("IMU_DATA: {0}".format(data))
                (data["pressureValid"], data["pressure"], data["temperatureValid"], data["temperature"]) = self.pressure.pressureRead()
                #fusionPose = data["fusionPose"]
                #self.get_logger().info("r: {0} p: {1} y: {2}".format(
                #    math.degrees(fusionPose[0]), 
                #    math.degrees(fusionPose[1]),
                #    math.degrees(fusionPose[2])
                #))
                
                # TODO : publish this info too for GPS fusion
                # TODO: the get_altitude should get local pressure in hPa
                # example: inhg of 30.1 to hpa was 1019.30
                if data["pressureValid"]:
                    self.get_logger().info("Pressure: {0}, height above sea level: {1}".format(
                        data["pressure"],
                        get_altitude(data["pressure"], sea_level_hPa=1019.30)
                    ))
                
                if data["temperatureValid"]:
                    self.get_logger().info("Temperature C: {0} Temperature F: {1}".format(data["temperature"], (data["temperature"] * (9/5)) + 32 ))
                
                # build msg and publish
                qfp = data['fusionQPose']
                a = data['accel']
                g = data['gyro']
                
                ori = Quaternion(x=qfp[1], y=qfp[2], z=qfp[3], w=qfp[0])
                av = Vector3(x=g[0], y=g[1], z=g[2])
                la = Vector3(x=a[1] * G_TO_MPSS, y=a[1] * G_TO_MPSS, z=a[2] * G_TO_MPSS)

                imu_msg = Imu(orientation=ori, angular_velocity=av, linear_acceleration=la)
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = self.frame_id

                self.imu_pub.publish(imu_msg)


def main():
    rclpy.init()
    imu_node = IMUNode()
    try:
        rclpy.spin(imu_node)
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

