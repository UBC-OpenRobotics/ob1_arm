#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Arduino.h>
#include "Stepper.h"
#include <ros.h>
#include <std_msgs/Float64.h>
#include "armCmd.h"
#include "armState.h"

//##################################
//defines
//##################################

// bicep 
#define stepPin_1 3
#define dirPin_1 4
#define steps_per_rev_1 4000
// bicep 
#define stepPin_2 5
#define dirPin_2 6
#define steps_per_rev_2 4000
// shoulder 
#define stepPin_3 7
#define dirPin_3 8
#define steps_per_rev_3 4000
//
#define stepPin_4 9
#define dirPin_4 10
#define steps_per_rev_4 4000
//
#define stepPin_5 11
#define dirPin_5 12
#define steps_per_rev_5 4000
//
#define stepPin_6 13
#define dirPin_6 14
#define steps_per_rev_6 4000

//function prototypes
void armCmdCb( const ob1_arm_hw_interface::armCmd &msg);

//##################################
//globals
//##################################
ros::NodeHandle nh;

uint32_t rcv_cnt;
uint32_t last_rcv_cnt;
int num_joints;

ob1_arm_hw_interface::armCmd armCmd;
ob1_arm_hw_interface::armState armState;
ros::Publisher p("/arduino/armState", &armState);

ros::Subscriber<ob1_arm_hw_interface::armCmd> s("/arduino/armCmd", &armCmdCb);

  
// instantiate steppers
AccelStepper stepper1(AccelStepper::DRIVER, stepPin_1, dirPin_1);
AccelStepper stepper2(AccelStepper::DRIVER, stepPin_2, dirPin_2);
AccelStepper stepper3(AccelStepper::DRIVER, stepPin_3, dirPin_3);
AccelStepper stepper4(AccelStepper::DRIVER, stepPin_4, dirPin_4);
AccelStepper stepper5(AccelStepper::DRIVER, stepPin_5, dirPin_5);
AccelStepper stepper6(AccelStepper::DRIVER, stepPin_6, dirPin_6);

MultiStepper steppers;

//##################################
//functions
//##################################

void setup() {
  // set up serial jic we need to monitor anything
  Serial.begin(57600); // read
  // stepper constructor should take care of all the pin initializations
  rcv_cnt = 0;
  last_rcv_cnt = 0;
  num_joints = 0;
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(s);
  nh.negotiateTopics();

  // set parameters for the steppers
  stepper1.setMaxSpeed(200.0);
  stepper2.setMaxSpeed(200.0);
  stepper3.setMaxSpeed(200.0);
  stepper4.setMaxSpeed(200.0);
  stepper5.setMaxSpeed(200.0);
  stepper6.setMaxSpeed(200.0);

  stepper1.setAcceleration(100.0);
  stepper2.setAcceleration(100.0);
  stepper3.setAcceleration(100.0);
  stepper4.setAcceleration(100.0);
  stepper5.setAcceleration(100.0);
  stepper6.setAcceleration(100.0);

  //add steppers to multistepper 
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);
  steppers.addStepper(stepper4);
  steppers.addStepper(stepper5);
  steppers.addStepper(stepper6);
}

void loop() {

  /*
  Write serial to send current joint positions
  */

  if (rcv_cnt > last_rcv_cnt) {
    last_rcv_cnt++;
    long position_goal[6]; 
    //needs to make the msg into the proper type?
    for(int i = 0; i < num_joints; i++){
      position_goal[i] = armState.angle[i];
    }
    // move joints based on position_goal
    steppers.moveTo(position_goal); //set position goal for each motor based off of the ROS message

    //set velocity goal for each movement as specified in ros msg
    stepper1.setSpeed(armState.vel[0]);
    stepper2.setSpeed(armState.vel[1]); 
    stepper3.setSpeed(armState.vel[2]);
    stepper4.setSpeed(armState.vel[3]);
    stepper5.setSpeed(armState.vel[4]); 
    stepper6.setSpeed(armState.vel[5]);

    //run steppers based on set speed
    steppers.runSpeedToPosition(); // Blocks until all are in position
    }
  
    nh.spinOnce();
}

//callback function called when new arm cmd received
void armCmdCb( const ob1_arm_hw_interface::armCmd &msg){
  armCmd = msg;
  rcv_cnt++;
  num_joints = msg.num_joints;
  
  armState.msg_rcv_ctr = rcv_cnt;
  for(int i = 0; i < num_joints; i++)
  {
    armState.angle[i] = msg.angle[0];
    armState.vel[i] = msg.vel[0];
  }

  p.publish( &armState );
}
