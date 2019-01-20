#include <pinDefsAutoNew.h>


#include <sra128.h>

//#include <ArduinoTcpHardware.h>
#include <ros.h>
#include <ArduinoHardware.h>

#include <Wire.h>
#include <std_msgs/Float32.h>
#include<std_msgs/Float32MultiArray.h>
#include<std_msgs/MultiArrayLayout.h>
#include<std_msgs/MultiArrayDimension.h>
#include<VL53L0X.h>

ros::NodeHandle nh;
VL53L0X sensor;
float tof_reading;

std_msgs::Float32MultiArray tofarray;
ros::Publisher pub_tof("Atmegasensors", &tofarray);



void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);

   nh.initNode();
  //nh.subscribe(sub);
  tofarray.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc (sizeof(std_msgs::MultiArrayDimension) *2);
  tofarray.layout.dim[0].label = "Height";
  tofarray.layout.dim[0].size = 8;
  tofarray.layout.dim[0].stride = 1*8;
  tofarray.layout.data_offset = 0;
  tofarray.data= (float *)malloc(sizeof(float)*8);
  tofarray.data_length= 4;
  nh.advertise(pub_tof);

   sensor.startContinuous();

 
}

long sensor_time;

void loop() {
  // put your main code here, to run repeatedly:
  tofarray.data[0] = 1;
  tofarray.data[1] = 2;
  tofarray.data[2] = 3;
  tofarray.data[3] = 4;
  pub_tof.publish(&tofarray);
  nh.spinOnce();
}
