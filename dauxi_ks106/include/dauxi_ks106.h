#ifndef KS106UART_H
#define KS106UART_H

#include <vector>
#include <time.h>

#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Range.h"

namespace IQR {
  class Ks106Uart {
  private:
    float distance_;
    uint8_t add_,reg_;
    int ks106_con_,freq_,detect_model_;
    
    std::string port_,nodename_;

    std::string  us_id_[4];
    serial::Serial ser_;
    int baudrate_,index_;
    std::vector<uint8_t> UartData_;
    std::string topic_pub_;

    ros::Publisher ks106_pub_[4];

    double min_range_, max_range_;
  public:
    Ks106Uart(ros::NodeHandle&);
    ~Ks106Uart();
    
    bool UartInit();
    bool Ks106Init();
    int Ks106Run();
    bool WriteData(uint8_t, double);
    int WriteDetect();
    bool ReadAndCheck();
    int PubDistance(bool);
    int WriteCommand();
    int Frequency();
  };
  const uint8_t detect_cmd_model1[5] = {0x30,0x32,0x34,0x36,0x37};
  const uint8_t command_model2[4] = {0x9c,0x95,0x98,0x75};
}

#endif