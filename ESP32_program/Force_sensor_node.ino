


//Libraries
#include "HX711.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh_force;

//Create msgs
geometry_msgs::Vector3 Force;

//Define publisher
ros::Publisher force_pub("/Force", &Force);


//Declare Pins
const int DTX = 17; 
const int DTY = 19;
const int DTZ = 0;

const int SCKX = 16;
const int SCKY = 18;
const int SCKZ = 2;


//Create 3 load cells
HX711 LCx;
HX711 LCy;
HX711 LCz;



// Calibrate sensor
float Calibrate_x(long bits){

  float Force_x = 2.0245e-5*bits - 166;

  return Force_x;
}
float Calibrate_y(long bits){
  
  float Force_y = 2.2905e-5 * bits - 194.89;

  return Force_y;
}
float Calibrate_z(long bits){
  
  float Force_z = -2.8686e-5 * bits +  243.19;

  return Force_z;
}


void setup() {
  
  //Initialize node
  nh_force.initNode();
  nh_force.advertise(force_pub);

  //Configure monitor serie
  //Serial.begin(115200);
  //Initialize load cells
  LCx.begin(DTX,SCKX);
  LCy.begin(DTY,SCKY);
  LCz.begin(DTZ,SCKZ);

  //Tare load cells 
  LCx.tare();
  LCy.tare();
  LCz.tare();
}

void loop() {

  //Start reading the number of bits
  long bits_x = LCx.read();
  long bits_y = LCy.read();
  long bits_z = LCz.read();

  //Convert bits to N
  Force.x = Calibrate_x(bits_x);
  Force.y = Calibrate_y(bits_y);
  Force.z = Calibrate_z(bits_z);

  force_pub.publish(&Force);

  delay(100); //0.1 s //10Hz

  nh_force.spinOnce();
}

