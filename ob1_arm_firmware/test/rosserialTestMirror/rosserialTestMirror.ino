
#include <ros.h>
#include <std_msgs/Float64.h>
#include "armCmd.h"
#include "armState.h"


ros::NodeHandle nh;

uint32_t rcv_cnt;
int num_joints;

ob1_arm_hw_interface::armState armState;
ros::Publisher p("/arduino/armState", &armState);

//callback function called when new arm cmd received
void armCmdCb( const ob1_arm_hw_interface::armCmd &msg){
  rcv_cnt++;
  num_joints = msg.num_joints;
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led on rcv
  
  armState.msg_rcv_ctr = rcv_cnt;
  for(int i = 0; i < num_joints; i++)
  {
    armState.angle[i] = msg.angle[0];
    armState.vel[i] = msg.vel[0];
  }

   p.publish( &armState );
  
}

ros::Subscriber<ob1_arm_hw_interface::armCmd> s("/arduino/armCmd", &armCmdCb);

void setup()
{
  Serial.begin(57600);
  rcv_cnt = 0;
  num_joints = 0;
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(s);
}

void loop()
{  
  nh.spinOnce();
  delay(10);
}
