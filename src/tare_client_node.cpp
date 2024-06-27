#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include "force_sensor_3D/tare_srv.h"

float pdtex = 2.6025e-5;
float pdtey = 2.3369e-5;
float pdtez = -2.887e-5;

int main(int argc, char **argv) {
  ros::init(argc, argv, "tare_client_node");
  ros::NodeHandle nh;

  //Create publisher for bias
  ros::Publisher bias_pub = nh. advertise<geometry_msgs::Vector3>("/Tare",10);

  // Create client
  ros::ServiceClient tare_client = nh.serviceClient<force_sensor_3D::tare_srv>("Tare_Service");

  // Create service
  force_sensor_3D::tare_srv tare_srv;


  if (tare_client.call(tare_srv)) {
    //Read response
    long bits_x = tare_srv.response.bits_x;
    long bits_y = tare_srv.response.bits_y;
    long bits_z = tare_srv.response.bits_z;

    // Calculate bias
    float bias_x = -bits_x * pdtex;
    float bias_y = -bits_y * pdtey;
    float bias_z = -bits_z * pdtez;

    //Create message for publisher
    geometry_msgs::Vector3 bias_msg;

    bias_msg.x = bias_x;
    bias_msg.y = bias_y;
    bias_msg.z = bias_z;

    bias_pub.publish(bias_msg);
    
  } else {
    ROS_ERROR("Failed to call service tare_srv");
    return 1;
  }

  ros::shutdown();

  return 0;
}
