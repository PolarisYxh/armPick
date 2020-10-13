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

    def convert_imu_data(self, topic_name):
        print 'start convert %d %s...' %(self.__ros_bag.get_message_count(topic_name), topic_name)

        csv_file = open(self.__csv_name_prefix+ '_' + topic_name.replace('/', '_') +'.csv', 'w')

        headers = [ 'timestamp',
                    'yaw', 'pitch', 'roll',
                    'linear_accel_x', 'linear_accel_y', 'linear_accel_z',
                    'angular_vel_x', 'angular_vel_y', 'angular_vel_z']

        csv_writer = csv.DictWriter(csv_file, headers)
        csv_writer.writeheader()

        for topic, msg, timestamp in self.__ros_bag.read_messages(topics=[topic_name]):
            (r, p, y) = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

            csv_writer.writerow({'timestamp': self.__get_time_delta(self.__ros_bag_start_time, timestamp.to_time()),
                                        'yaw': y, 'pitch': p, 'roll': r,
                                        'linear_accel_x': msg.linear_acceleration.x, 'linear_accel_y': msg.linear_acceleration.y, 'linear_accel_z': msg.linear_acceleration.z,
                                        'angular_vel_x': msg.angular_velocity.x, 'angular_vel_y': msg.angular_velocity.y, 'angular_vel_z': msg.angular_velocity.z})
        print('finish')

    def convert_cmd_data(self, topic_name):
        print 'start convert %d %s...' %(self.__ros_bag.get_message_count(topic_name), topic_name)

        csv_file = open(self.__csv_name_prefix+ '_' + topic_name.replace('/', '_') +'.csv', 'w')

        headers = [ 'timestamp',
                    'linear_x', 'linear_y', 'linear_z',
                    'angular_x', 'angular_y', 'angular_z']

        csv_writer = csv.DictWriter(csv_file, headers)
        csv_writer.writeheader()

        for topic, msg, timestamp in self.__ros_bag.read_messages(topics=[topic_name]):
            csv_writer.writerow({'timestamp': self.__get_time_delta(self.__ros_bag_start_time, timestamp.to_time()),
                                        'linear_x': msg.linear.x, 'linear_y': msg.linear.y, 'linear_z': msg.linear.z,
                                        'angular_x': msg.angular.x, 'angular_y': msg.angular.y, 'angular_z': msg.angular.z})
        print('finish')

    def convert_diagnostics_data(self, topic_name):
        print 'start convert %d %s...' %(self.__ros_bag.get_message_count(topic_name), topic_name)

        csv_file = open(self.__csv_name_prefix+ '_' + topic_name.replace('/', '_') +'.csv', 'w')

        headers = [ 'timestamp',
                    'l_voltage', 'l_current',
                    'r_voltage', 'r_current']

        csv_writer = csv.DictWriter(csv_file, headers)
        csv_writer.writeheader()

        for topic, msg, timestamp in self.__ros_bag.read_messages(topics=[topic_name]):
            for status in msg.status:
                if status.name == 'husky_node: system_status':
                    csv_writer.writerow({   'timestamp': self.__get_time_delta(self.__ros_bag_start_time, timestamp.to_time()),
                                            'l_voltage': status.values[2].value, 'l_current': status.values[5].value,
                                            'r_voltage': status.values[3].value, 'r_current': status.values[6].value})
        print('finish')

def main():
    parse = optparse.OptionParser()
    parse.add_option('-r', '--rosbag', dest='rosbag_name', help = 'rosbag name to convert')
    parse.add_option('-c', '--csv_prefix', dest='csv_prefix', help = 'target csv file prefix')

    (options, args) = parse.parse_args()

    t = ROSBagConvert(options.rosbag_name, options.csv_prefix)
    t.convert_imu_data('/imu_data')
    t.convert_imu_data('/imu_data_head')
    t.convert_cmd_data('/cmd_vel')
    t.convert_diagnostics_data('/diagnostics')

if __name__ == "__main__":
    main()
