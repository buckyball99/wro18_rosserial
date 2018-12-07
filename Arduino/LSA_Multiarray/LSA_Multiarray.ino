#include<sra128.h>
#include <ros.h> 

#include <pinDefsAutoNew.h>
#include<std_msgs/MultiArrayLayout.h>
#include<std_msgs/MultiArrayDimension.h>
#include<std_msgs/Float32MultiArray.h>

int junction, readVal , readValback, junctionback, junctionno,positionVal, junctionnoback,positionValback;

ros::NodeHandle nh;
  void calculate_data(){
    readVal = adc_start(0);
  junction = adc_start(1);
   readValback = adc_start(2);
  junctionback = adc_start(3);


  junctionno = (((float)junction / 921) * 70);
  positionVal = (((float)readVal / 921) * 70);
junctionnoback = (((float)junction / 921) * 70);
  positionValback = (((float)readValback / 921) * 70);
  }
  
  std_msgs::Float32MultiArray lsaarray;
  ros::Publisher pub_lsa("Atmegasensors",&lsaarray);

void setup() {
  Serial.begin(115200);
  nh.initNode();
  adc_init();
  lsaarray.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc (sizeof(std_msgs::MultiArrayDimension) *2);
  lsaarray.layout.dim[0].label = "Height";
  lsaarray.layout.dim[0].size = 8;
  lsaarray.layout.dim[0].stride = 1*8;
  lsaarray.layout.data_offset = 0;
  lsaarray.data= (float *)malloc(sizeof(float)*8);
  lsaarray.data_length= 8;
  nh.advertise(pub_lsa);

  
}

void loop(){
   calculate_data();
    for(int j =0; j<2; j++){
      
        lsaarray.data[j]= junctionno;
      
      }

      for(int k =2; k<4; k++){
      
        lsaarray.data[k]= positionVal;
      
      }

      for(int l=4; l<6; l++){
      
        lsaarray.data[l]= junctionnoback;
      
      }

      for(int m =6; m<8; m++){
      
        lsaarray.data[m]= positionValback;
      
      }

    pub_lsa.publish(&lsaarray);
    nh.spinOnce();

  
}









