#include "ros/ros.h"
#include "dauxi_ks106.h"

int main(int argc,char** argv) {

  ros::init(argc,argv,"ks106_node");
  ros::NodeHandle private_nh("~");
  IQR::Ks106Uart ks106(private_nh);

  //declare the uart port
  while(ks106.UartInit() == false) {
    sleep(1);
  }

  //add check code, set ks106 with launch setting
  ros::Rate loop_rate(ks106.Frequency());
  while(ros::ok()) {
    ks106.Ks106Run();
    loop_rate.sleep();
  }

  return 0;
}