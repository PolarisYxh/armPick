/*
 * ros driver node for JY901 imu
 * guolindong@gmail.com
 * 2018.01.23
 */

#include <math.h>
#include <iostream>

#include <ros/ros.h>
#include <serial/serial.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include "tf/LinearMath/Quaternion.h"

#include "JY901.h"

struct STime    stcTime;
struct SAcc     stcAcc;
struct SGyro    stcGyro;
struct SAngle   stcAngle;
struct SMag     stcMag;
struct SDStatus stcDStatus;
struct SPress   stcPress;
struct SLonLat  stcLonLat;
struct SGPSV    stcGPSV;
struct SOrien   stcOrien;
struct GPSStatus stcGPSStatus;

unsigned char chrTemp[5000];
unsigned char ucRxCnt = 0;
unsigned int usRxLength = 0;

void DoParse() {
  static int16_t temp_16;
  static int32_t temp_32;

  std::cout << std::fixed << std::setprecision(8);
  switch(chrTemp[1]) {
    case 0x50: {
      stcTime.ucYear = static_cast<int>(chrTemp[2]) + 2000;
      stcTime.ucMonth = chrTemp[3];
      stcTime.ucDay = chrTemp[4];
      stcTime.ucHour = chrTemp[5];
      stcTime.ucMinute = chrTemp[6];
      stcTime.ucSecond = chrTemp[7];
      stcTime.usMiliSecond = (chrTemp[9]<<8)|chrTemp[8];

      // std::cout << "Time: " << stcTime.ucYear << "-" << static_cast<int>(stcTime.ucMonth) << "-" << static_cast<int>(stcTime.ucDay) << " " << static_cast<int>(stcTime.ucHour) << ":" << static_cast<int>(stcTime.ucMinute) << ":" << static_cast<int>(stcTime.ucSecond) << ":" << static_cast<int>(stcTime.usMiliSecond) << std::endl;
      break;
    }
    case 0x51: {
      temp_16 = (static_cast<int16_t>(chrTemp[3])<<8)|chrTemp[2];
      stcAcc.a[0] = static_cast<double>(temp_16)/32768.0*16.0*9.80665;
      temp_16 = (static_cast<int16_t>(chrTemp[5])<<8)|chrTemp[4];
      stcAcc.a[1] = static_cast<double>(temp_16)/32768.0*16.0*9.80665;
      temp_16 = (static_cast<int16_t>(chrTemp[7])<<8)|chrTemp[6];
      stcAcc.a[2] = static_cast<double>(temp_16)/32768.0*16.0*9.80665;
      temp_16 = (static_cast<int16_t>(chrTemp[9])<<8)|chrTemp[8];
      stcAcc.T = static_cast<double>(temp_16)/100.0;

      // std::cout << "Temp: " << stcAcc.T << ", " << "Acc: " << stcAcc.a[0] << ", " << stcAcc.a[1] << ", " << stcAcc.a[2] << std::endl;
      break;
    }
    case 0x52: {
      temp_16 = (static_cast<int16_t>(chrTemp[3])<<8)|chrTemp[2];
      stcGyro.w[0] = static_cast<double>(temp_16)/32768.0*2000.0*M_PI/180.0;
      temp_16 = (static_cast<int16_t>(chrTemp[5])<<8)|chrTemp[4];
      stcGyro.w[1] = static_cast<double>(temp_16)/32768.0*2000.0*M_PI/180.0;
      temp_16 = (static_cast<int16_t>(chrTemp[7])<<8)|chrTemp[6];
      stcGyro.w[2] = static_cast<double>(temp_16)/32768.0*2000.0*M_PI/180.0;
      temp_16 = (static_cast<int16_t>(chrTemp[9])<<8)|chrTemp[8];
      stcAcc.T = static_cast<double>(temp_16)/100.0;

      // std::cout << "Temp: " << stcGyro.T << ", " << "Gyro: " << stcGyro.w[0] << ", " << stcGyro.w[1] << ", " << stcGyro.w[2] << std::endl;
      break;
    }
    case 0x53: {
      temp_16 = (static_cast<int16_t>(chrTemp[3])<<8)|chrTemp[2];
      stcAngle.Angle[0] = static_cast<double>(temp_16)/32768.0*M_PI;
      temp_16 = (static_cast<int16_t>(chrTemp[5])<<8)|chrTemp[4];
      stcAngle.Angle[1] = static_cast<double>(temp_16)/32768.0*M_PI;
      temp_16 = (static_cast<int16_t>(chrTemp[7])<<8)|chrTemp[6];
      stcAngle.Angle[2] = static_cast<double>(temp_16)/32768.0*M_PI;
      temp_16 = (static_cast<int16_t>(chrTemp[9])<<8)|chrTemp[8];
      stcAngle.T = static_cast<double>(temp_16)/100.0;

      // std::cout << "Temp: " << stcAngle.T << ", " << "Angle: " << stcAngle.Angle[0] << ", " << stcAngle.Angle[1] << ", " << stcAngle.Angle[2] << std::endl;
      break;
    }
    case 0x54: {
      temp_16 = (static_cast<int16_t>(chrTemp[3])<<8)|chrTemp[2];
      stcMag.h[0] = static_cast<double>(temp_16)/1000.0/10000.0;
      temp_16 = (static_cast<int16_t>(chrTemp[5])<<8)|chrTemp[4];
      stcMag.h[1] = static_cast<double>(temp_16)/1000.0/10000.0;
      temp_16 = (static_cast<int16_t>(chrTemp[7])<<8)|chrTemp[6];
      stcMag.h[2] = static_cast<double>(temp_16)/1000.0/10000.0;
      temp_16 = (static_cast<int16_t>(chrTemp[9])<<8)|chrTemp[8];
      stcMag.T = static_cast<double>(temp_16)/100.0;

      // std::cout << "Temp: " << stcMag.T << ", " << "Magnetic: " << stcMag.h[0] << ", " << stcMag.h[1] << ", " << stcMag.h[2] << std::endl;
      break;
    }
    case 0x55: {
      /* I/O value, not in use */
      break;
    }
    case 0x56: {
      temp_32 = (static_cast<int32_t>(chrTemp[5])<<24)|(static_cast<int32_t>(chrTemp[4])<<16)|(static_cast<int32_t>(chrTemp[3])<<8)|chrTemp[2];
      stcPress.lPressure = static_cast<double>(temp_32);
      temp_32 = (static_cast<int32_t>(chrTemp[9])<<24)|(static_cast<int32_t>(chrTemp[8])<<16)|(static_cast<int32_t>(chrTemp[7])<<8)|chrTemp[6];
      stcPress.lAltitude = static_cast<double>(temp_32) / 100.0;

      // std::cout << "Pressure: " << stcPress.lPressure << ", " << "Altitude: " << stcPress.lAltitude << std::endl;
      break;
    }
    case 0x57: {
      temp_32 = (static_cast<int32_t>(chrTemp[5])<<24)|(static_cast<int32_t>(chrTemp[4])<<16)|(static_cast<int32_t>(chrTemp[3])<<8)|chrTemp[2];
      stcLonLat.lLon = temp_32/10000000 + (temp_32%10000000)/100000.0/60.0;
      temp_32 = (static_cast<int32_t>(chrTemp[9])<<24)|(static_cast<int32_t>(chrTemp[8])<<16)|(static_cast<int32_t>(chrTemp[7])<<8)|chrTemp[6];
      stcLonLat.lLat = temp_32/10000000 + (temp_32%10000000)/100000.0/60.0;
      // std::cout << "GPS: " << stcLonLat.lLon << ", " << stcLonLat.lLat << std::endl;
      break;
    }
    case 0x58: {
      temp_16 = (static_cast<int16_t>(chrTemp[3])<<8)|chrTemp[2];
      stcGPSV.fGPSHeight = static_cast<double>(temp_16) / 10.0;
      temp_16 = (static_cast<int16_t>(chrTemp[5])<<8)|chrTemp[4];
      stcGPSV.fGPSYaw = static_cast<double>(temp_16) / 10.0;
      temp_32 = (static_cast<int32_t>(chrTemp[9])<<24)|(static_cast<int32_t>(chrTemp[8])<<16)|(static_cast<int32_t>(chrTemp[7])<<8)|chrTemp[6];
      stcGPSV.fGPSVelocity = static_cast<double>(temp_32) / 1000.0;

      // std::cout << "Height: " << stcGPSV.fGPSHeight << ", Yaw: " << stcGPSV.fGPSYaw << ", Velocity: " << stcGPSV.fGPSVelocity << std::endl;
      break;
    }
    case 0x59: {
      temp_16 = (static_cast<int16_t>(chrTemp[3])<<8)|chrTemp[2];
      stcOrien.q[0] = static_cast<double>(temp_16)/32768.0;
      temp_16 = (static_cast<int16_t>(chrTemp[5])<<8)|chrTemp[4];
      stcOrien.q[1] = static_cast<double>(temp_16)/32768.0;
      temp_16 = (static_cast<int16_t>(chrTemp[7])<<8)|chrTemp[6];
      stcOrien.q[2] = static_cast<double>(temp_16)/32768.0;
      temp_16 = (static_cast<int16_t>(chrTemp[9])<<8)|chrTemp[8];
      stcOrien.q[3] = static_cast<double>(temp_16)/32768.0;

      // std::cout << "Orientation: " << stcOrien.q[0] << ", " << stcOrien.q[1] << ", " << stcOrien.q[2] << ", " << stcOrien.q[3] << std::endl;
      break;
    }
    case 0x5A: {
      temp_16 = (static_cast<int16_t>(chrTemp[3])<<8)|chrTemp[2];
      stcGPSStatus.sat_num = static_cast<int>(temp_16);
      temp_16 = (static_cast<int16_t>(chrTemp[5])<<8)|chrTemp[4];
      stcGPSStatus.pdop = static_cast<double>(temp_16)/100.0;
      temp_16 = (static_cast<int16_t>(chrTemp[7])<<8)|chrTemp[6];
      stcGPSStatus.hdop = static_cast<double>(temp_16)/100.0;
      temp_16 = (static_cast<int16_t>(chrTemp[9])<<8)|chrTemp[8];
      stcGPSStatus.vdop = static_cast<double>(temp_16)/100.0;

      // std::cout << "Satellite number: " << stcGPSStatus.sat_num << ", Pdop:" << stcGPSStatus.pdop << ", Hdop:" << stcGPSStatus.hdop << ", Vdop:" << stcGPSStatus.vdop << std::endl;
      break;
    }
  }
}

// convert serial data to jy901 data
void CopeSerialData(std::string str_in) {

  // ROS_INFO("%i", str_in.size());


  unsigned int str_length = str_in.size();

  static int sum;
  memcpy(chrTemp+usRxLength, str_in.data(), str_length);
  usRxLength += str_length;
  while (usRxLength >= 11) {
    if (chrTemp[0] != 0x55) {
      usRxLength--;
      memcpy(&chrTemp[0], &chrTemp[1], usRxLength);
    } else {
      sum = 0;
      for(int i=0; i<10; i++) {
        sum += chrTemp[i];
      }
      if((int(sum & 0x00ff)==int(chrTemp[10]))) {
        DoParse();
        usRxLength -= 11;
        memcpy(&chrTemp[0], &chrTemp[11], usRxLength);

        // std::cout << std::hex << static_cast<int>(sum & 0xff) << ", ";
        // std::cout << std::hex << static_cast<int>(chrTemp[10]) << ", ";
        // std::cout << std::hex << (int(sum & 0x00ff)==int(chrTemp[10])) << std::endl;
      } else {
        usRxLength--;
        memcpy(&chrTemp[0], &chrTemp[1], usRxLength);
      }
    }
  }
}

int main (int argc, char** argv) {
  // param
  serial::Serial serial_port;
  std::string port;
  int baudrate;
  int looprate;

  // ros init
  ros::init(argc, argv, "jy901_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // get param from launch file
  pnh.param<int>("baudrate", baudrate, 115200);
  pnh.param<std::string>("port", port, "/dev/jydriver");
  pnh.param<int>("looprate", looprate, 100);

  pnh.getParam("baudrate", baudrate);
  pnh.getParam("port", port);
  pnh.getParam("looprate", looprate);

  ROS_INFO_STREAM(port);
  ROS_INFO_STREAM(baudrate);
  ROS_INFO_STREAM(looprate);

  // ros pub and sub
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 100);
  ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("gps/fix", 100);

  try {
    serial_port.setPort(port);
    serial_port.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(10);
    serial_port.setTimeout(to);
    serial_port.open();
    serial_port.setRTS(false);
    serial_port.setDTR(false);
    // serial_port.open();
  } catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open serial port ");
    return -1;
  }

  // check if serial port is open
  if(serial_port.isOpen()) {
    ROS_INFO_STREAM("Serial Port initialized");
  } else {
    return -1;
  }

  ros::Rate loop_rate(looprate);
  while(ros::ok()) {
    // convert serial string to JY901 data
    CopeSerialData(serial_port.read(200));

    // imu sensor msg pub
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu_link";

    tf::Quaternion quate;
    quate.setRPY(stcAngle.Angle[0], stcAngle.Angle[1], stcAngle.Angle[2]);
    imu_msg.orientation.w = quate.w();
    imu_msg.orientation.x = quate.x();
    imu_msg.orientation.y = quate.y();
    imu_msg.orientation.z = quate.z();
    imu_msg.orientation_covariance[0] = 0;
    imu_msg.linear_acceleration.x = stcAcc.a[0];
    imu_msg.linear_acceleration.y = stcAcc.a[1];
    imu_msg.linear_acceleration.z = stcAcc.a[2];
    imu_msg.linear_acceleration_covariance[0] = 0;
    imu_msg.angular_velocity.x = static_cast<float>(stcGyro.w[0]);
    imu_msg.angular_velocity.y = static_cast<float>(stcGyro.w[1]);
    imu_msg.angular_velocity.z = static_cast<float>(stcGyro.w[2]);
    imu_msg.angular_velocity_covariance[0] = 0;
    imu_pub.publish(imu_msg);
    
    // gps msg pub
    sensor_msgs::NavSatFix nav_sat_fix;
    nav_sat_fix.header.stamp = ros::Time::now();
    nav_sat_fix.header.frame_id = "imu_link";
    nav_sat_fix.status.status = -1;
    nav_sat_fix.status.service = 1;
    nav_sat_fix.longitude = stcLonLat.lLon;
    nav_sat_fix.latitude = stcLonLat.lLat;
    nav_sat_fix.altitude = stcGPSV.fGPSHeight;
    nav_sat_fix.position_covariance[0] = -1;
    nav_sat_fix.position_covariance_type = 0;
    gps_pub.publish(nav_sat_fix);

    ros::spinOnce();
    loop_rate.sleep();
  }
}
