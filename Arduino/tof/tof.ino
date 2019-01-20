/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
//#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include <Wire.h>
#include <VL53L0X.h>

ros::NodeHandle  nh;
VL53L0X sensor;
float tof_reading;


std_msgs::Float32 x;

/*void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
}
*/
ros::Publisher pub_tof("tof", &x);
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

void loop()
{  
  tof_reading = sensor.readRangeContinuousMillimeters();
  x.data = tof_reading;
  pub_tof.publish(&x);
  nh.spinOnce();
  delay(1);
}

