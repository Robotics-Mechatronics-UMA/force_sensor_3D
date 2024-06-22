// Libraries
#include "HX711.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <Tare_Sensor/Tare_srv.h>

ros::NodeHandle nh;

using Tare_srv = Tare_Sensor::Tare_srv;

// Create message
geometry_msgs::Vector3 Force;

// Declare Pins
const int DTX = 17;
const int DTY = 19;
const int DTZ = 0;

const int SCKX = 16;
const int SCKY = 18;
const int SCKZ = 2;

// Slopes
const float pdtex = 2.6025e-5;
const float pdtey = 2.3369e-5;
const float pdtez = -2.887e-5;

// Initial values
float bias_x = 0;
float bias_y = 0;
float bias_z = 0;

//
long bits_x = 0;
long bits_y = 0;
long bits_z = 0;

// Create 3 load cells
HX711 LCx;
HX711 LCy;
HX711 LCz;



// Timer Global variables
hw_timer_t *timer = NULL;
volatile bool timer_flag = false; // Timer flag


//Tare service Callback
void Tare_Callback(const Tare_srv::Request &req, Tare_srv::Response &res){

  //Read bits 
  long bits_x_tare = LCx.read_average(10);
  long bits_y_tare = LCy.read_average(10);
  long bits_z_tare = LCz.read_average(10);

  res.bits_x = bits_x_tare; 
  res.bits_y = bits_y_tare; 
  res.bits_z = bits_z_tare;

  nh.loginfo("Waiting for tare ...");
  
}

// Function to convert bits to force
float bits2force_x(long bits, float bias) {
  float Force_x = pdtex * bits + bias;
  return Force_x;
}

float bits2force_y(long bits, float bias) {
  float Force_y = pdtey * bits + bias;
  return Force_y;
}

float bits2force_z(long bits, float bias) {
  float Force_z = pdtez * bits + bias;
  return Force_z;
}


// Bias callback
void biasCallback(const geometry_msgs::Vector3 &bias_msg) {
  bias_x = bias_msg.x;
  bias_y = bias_msg.y;
  bias_z = bias_msg.z;

  nh.loginfo("Tare process has been succesfully completed");
}


//Define publisher
ros::Publisher force_pub("/Force", &Force);

////Define tare subscriber
ros::Subscriber<geometry_msgs::Vector3> bias_sub("/Tare", &biasCallback);

ros::ServiceServer<Tare_srv::Request, Tare_srv::Response> tare_srv("Tare_Service", &Tare_Callback);



// Timer interrupt handler
void timerInterrupt() {
  timer_flag = true;
}

void setup() {
  // Initialize node
  nh.initNode();
  
  // Initialize publisher and subscriber
  nh.advertise(force_pub);
  nh.subscribe(bias_sub);

  //Initialize service
  nh.advertiseService(tare_srv);

  // Set baudrate to 115200
  nh.getHardware()->setBaud(115200);
  
  // Initialize load cells
  LCx.begin(DTX, SCKX);
  LCy.begin(DTY, SCKY);
  LCz.begin(DTZ, SCKZ);

  // Read average bits for initial orientation
  long bitsx_tare = LCx.read_average(10);
  long bitsy_tare = LCy.read_average(10);
  long bitsz_tare = LCz.read_average(10);

  // Initial bias (initial tare)
  bias_x = -bitsx_tare * pdtex;
  bias_y = -bitsy_tare * pdtey;
  bias_z = -bitsz_tare * pdtez;

  // Configure Timer0 for 10 Hz interrupt
  timer = timerBegin(0, 80, true); // Timer0 with prescaler 80 (80MHz-ESP32)
  timerAttachInterrupt(timer, &timerInterrupt, true); // Attach interrupt handler
  timerAlarmWrite(timer, 100000, true); // 100000us (10ms), true for periodic alarm
  timerAlarmEnable(timer); // Enable timer
  

}

void loop() {
  
  if (timer_flag) {
    // Read bits from load cells
    bits_x = LCx.read();
    bits_y = LCy.read();
    bits_z = LCz.read();
  
    // Convert bits to force
    Force.x = bits2force_x(bits_x, bias_x);
    Force.y = bits2force_y(bits_y, bias_y);
    Force.z = bits2force_z(bits_z, bias_z);
    // Publish force
    force_pub.publish(&Force);
    timer_flag = false; // Restart timer flag
  }

  // Handle ROS communication
  nh.spinOnce();
}