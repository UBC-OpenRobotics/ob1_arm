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
#define pulse_1 3
#define dir_1 4
// bicep 
#define pulse_2 5
#define dir_2 6
// shoulder 
#define pulse_3 7
#define dir_3 8

// enable all steppers
#define enable 13
// steps per rev:
#define STEPSPERREV 4000


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

int PULpin[3] = {pulse_1, pulse_2, pulse_3};
int DIRpin[3] = {dir_1, dir_2, dir_3};

// make 3 stepper objects
// Stepper.Stepper(int _enable, int _pulse, int _dir, int _steps) 

Stepper mySteppers[3] = {Stepper(enable, PULpin[0], DIRpin[0], STEPSPERREV),
                         Stepper(enable, PULpin[1], DIRpin[1], STEPSPERREV),
                         Stepper(enable, PULpin[2], DIRpin[2], STEPSPERREV)};

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
}

void loop() {

  /*
    TODO: 
      * set up encoders --> they will be posn_actual's
      * serial input --> input a nx6 array of joint angles (degrees)
      *   currently this is produced by the input_sim.py script
      *   one at a time (they are all one degree at a time)
  */

  /*
  Write serial to send current joint positions
  */

  if (rcv_cnt > last_rcv_cnt) {
    last_rcv_cnt++;
    mySteppers[0].stepCW(4000);
  
    // if(position_goal[0] != position_measured[0]){
    //   //if position is not where you want it to be, step one time
    //   if((position_goal[0] - position_measured[0]) > 0){
    //     mySteppers[0].stepCW(1);
    //   }
    //   if((position_goal[0] - position_measured[0]) < 0){
    //     mySteppers[0].stepCCW(1);
    //   }
    //   if((position_goal[0] - position_measured[0]) == 0){
    //     Serial.println("motor_1 is in desired posn");
    //   }
    // }
    // if(position_goal[1] != position_measured[1]){
    //   //if position is not where you want it to be, step one time
    //   if((position_goal[1] - position_measured[1]) > 0){
    //     mySteppers[1].stepCW(1);
    //   }
    //   if((position_goal[1] - position_measured[1]) < 0){
    //     mySteppers[1].stepCCW(1);
    //   }
    //   if((position_goal[1] - position_measured[1]) == 0){
    //     Serial.println("motor_2 is in desired posn");
    //   }
    // }

    // if(position_goal[2] != position_measured[2]){
    //   //if position is not where you want it to be, step one time
    //   if((position_goal[2] - position_measured[2]) > 0){
    //     mySteppers[2].stepCW(1);
    //   }
    //   if((position_goal[2] - position_measured[2]) < 0){
    //     mySteppers[2].stepCCW(1);
    //   }
    //   if((position_goal[2] - position_measured[2]) == 0){
    //     Serial.println("motor_3 is in desired posn");
    //   }
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
