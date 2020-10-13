#!/usr/bin/env python

import csv
import datetime
import optparse

import rosbag
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray

from tf.transformations import euler_from_quaternion


class ROSBagConvert():
    def __init__(self, ros_bag_name, csv_name_prefix):

        self.__ros_bag_name = ros_bag_name
        self.__ros_bag = rosbag.Bag(ros_bag_name)

        self.__ros_bag_start_time = self.__ros_bag.get_start_time()

        self.__csv_name_prefix = csv_name_prefix

    def __get_time_delta(self, start_timestamp, end_timestamp):
        start_time = datetime.datetime.fromtimestamp(start_timestamp)
        end_time = datetime.datetime.fromtimestamp(end_timestamp)
        delta_time = end_time - start_time
        delta_seconds = delta_time.seconds + delta_time.microseconds/1000000.0
        return delta_seconds

    def convert_imu_data(self, topics_name):
        for topic_name in topics_name:
            print 'start convert %d %s...' %(self.__ros_bag.get_message_count(topic_name), topic_name)

        csv_file = open('/home/caster/Documents/caster-tests/logs/all' +'.csv', 'w')
        headers = [ 'timestamp',
                    'yaw', 'pitch', 'roll',
                    'linear_accel_x', 'linear_accel_y', 'linear_accel_z',
                    'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
                    'linear_vel_x_base', 'angular_vel_z_base',
                    'battery_voltage', 'Current', 'Rsoc', 'NTC1', "NTC2",
                    'left_motor_speed', 'left_motor_current', 'right_motor_speed', 'right_motor_current',
                    'controller_mos_tem', 'controller_mcu_tem', 
                    'ultrasonic_front_left_range', 'ultrasonic_front_right_range', 'ultrasonic_rear_left_range', 'ultrasonic_rear_right_range']
        csv_writer = csv.DictWriter(csv_file, headers)
        csv_writer.writeheader()

        for topic, msg, timestamp in self.__ros_bag.read_messages(topics=topics_name):
            # print("%s", topic)            
            if topic == '/imu_data': 
                (r, p, y) = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

                csv_writer.writerow({'timestamp': self.__get_time_delta(self.__ros_bag_start_time, timestamp.to_time()),
                                            'yaw': y, 'pitch': p, 'roll': r,
                                            'linear_accel_x': msg.linear_acceleration.x, 'linear_accel_y': msg.linear_acceleration.y, 'linear_accel_z': msg.linear_acceleration.z,
                                            'angular_vel_x': msg.angular_velocity.x, 'angular_vel_y': msg.angular_velocity.y, 'angular_vel_z': msg.angular_velocity.z})

            if topic == '/cmd_vel':
                csv_writer.writerow({'timestamp': self.__get_time_delta(self.__ros_bag_start_time, timestamp.to_time()),
                                            'linear_vel_x_base': msg.linear.x, 'angular_vel_z_base': msg.angular.z})

            if topic == '/diagnostics':
                if msg.status[0].name == "caster_base_node: Left motor":
                    csv_writer.writerow({'timestamp': self.__get_time_delta(self.__ros_bag_start_time, timestamp.to_time()),
                                                  'left_motor_speed': msg.status[0].values[0].value, 'left_motor_current': msg.status[0].values[1].value,
                                                  'right_motor_speed': msg.status[1].values[0].value, 'right_motor_current': msg.status[1].values[1].value,
                                                  'controller_mos_tem': msg.status[3].values[0].value, 'controller_mcu_tem': msg.status[3].values[1].value})
                if msg.status[0].name == "hongfu_bms_status_node: BMS":
                    csv_writer.writerow({'timestamp': self.__get_time_delta(self.__ros_bag_start_time, timestamp.to_time()),
                                                  'battery_voltage': msg.status[0].values[0].value, 'Current': msg.status[0].values[1].value, 'Rsoc': msg.status[0].values[7].value,
                                                   'NTC1': msg.status[0].values[17].value, 'NTC2': msg.status[0].values[18].value})

            if topic == '/dauxi_ks106_node/ultrasonic_front_left':
                csv_writer.writerow({'timestamp': self.__get_time_delta(self.__ros_bag_start_time, timestamp.to_time()),
                                            'ultrasonic_front_left_range': msg.range})
            if topic == '/dauxi_ks106_node/ultrasonic_front_right':
                csv_writer.writerow({'timestamp': self.__get_time_delta(self.__ros_bag_start_time, timestamp.to_time()),
                                            'ultrasonic_front_right_range': msg.range})
            if topic == '/dauxi_ks106_node/ultrasonic_rear_left':
                csv_writer.writerow({'timestamp': self.__get_time_delta(self.__ros_bag_start_time, timestamp.to_time()),
                                            'ultrasonic_rear_left_range': msg.range})
            if topic == '/dauxi_ks106_node/ultrasonic_rear_right':
                csv_writer.writerow({'timestamp': self.__get_time_delta(self.__ros_bag_start_time, timestamp.to_time()),
                                            'ultrasonic_rear_right_range': msg.range})
        print('finish')


def main():
    parse = optparse.OptionParser()
    parse.add_option('-r', '--rosbag', dest='rosbag_name', help = 'rosbag name to convert')
    parse.add_option('-c', '--csv_prefix', dest='csv_prefix', help = 'target csv file prefix')

    (options, args) = parse.parse_args()

    t = ROSBagConvert(options.rosbag_name, options.csv_prefix)
    topics = ['/imu_data', '/cmd_vel', '/diagnostics', '/dauxi_ks106_node/ultrasonic_front_left', '/dauxi_ks106_node/ultrasonic_front_right', '/dauxi_ks106_node/ultrasonic_rear_left', '/dauxi_ks106_node/ultrasonic_rear_right']
    t.convert_imu_data(topics)

if __name__ == "__main__":
    main()
