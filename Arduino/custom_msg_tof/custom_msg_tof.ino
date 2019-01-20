#include <ArduinoTcpHardware.h>
#include <ros.h>
#include <ArduinoHardware.h>

/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <std_msgs/Empty.h>
#include <arduino2ros/TOF_sensor.h>
#include <std_msgs/FLoat32.h>
#include "TOF_sensor.h"
#include <Wire.h>
#include <VL53L0X.h>

ros::NodeHandle  nh;
VL53L0X sensor;
float tof_reading;


TOF_sensor::Float32 data;

/*void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
}
*/
ros::Publisher pub_tof("tof", &data);
//ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{ 
  pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);
  
  nh.initNode();
  //nh.subscribe(sub);
  nh.advertise(pub_tof);

   sensor.startContinuous();
}

long sensor_time;
void loop()
{  
  if( millis() >= sensor_time){
  tof_reading = sensor.readRangeContinuousMillimeters();
 data.dist = tof_reading;
  pub_tof.publish( &data);
  sensor_time = millis()+ 50;
}
  nh.spinOnce();

}

