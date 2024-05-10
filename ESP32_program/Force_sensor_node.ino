


//Libraries
#include "HX711.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh_force;

//Create msgs
geometry_msgs::Vector3 Bits;

//Define publisher
ros::Publisher force_pub("/Force", &Bits);


//Declare Pins
const int DTX = 19;
const int DTY = 17;
const int DTZ = 0;

const int SCKX = 18;
const int SCKY = 16;
const int SCKZ = 2;


//Create 3 load cells
HX711 LCx;
HX711 LCy;
HX711 LCz;

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

  Bits.x = bits_x;
  Bits.y = bits_y;
  Bits.z = bits_z;

  // //Convert bits to N
  // float Fx = CalculateForce(bits_x);
  // float Fy = CalculateForce(bits_y);
  // float Fz = CalculateForce(bits_z);

  force_pub.publish(&Bits);

  delay(10); //0.01 s //100Hz
}

// Linear regression of data in bits
// float CalculateForce(long bits){

//   //Slope m, intersection n
//   float Force = m * bits + n;

//   return Force;
// }