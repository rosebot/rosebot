#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include "XL320.h"
#include "PID_v1.h"
#include "AFMotor.h"
#include "AFMotor.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

XL320 neckPan;
const int neck_set_interval = 10000; //How often should the speed be set for the neck pan
int prev_neck_time = 0;
int neckPanSpeed = 35;

// Set some variables for incrementing position & LED colour
char RGB[] = "rgbypcwo";
const int COLORS[][3] = {{1,0,0},
                   {0,1,0},
                   {0,0,1},
                   {1,1,0},
                   {1,0,1},
                   {0,1,1},
                   {1,1,1},
                   {0,0,0}};

int servoPosition = 0;
int ledColour = 0;

// Set the servoID to talk to
int servoID = 6;

ros::NodeHandle nh;

// How often to run the control loops (ms)
const int loopTime = 25;

const int TOTAL_FACE_SERVOS = 5;
const int FACE_PINS[] = {23,25,27,29,31};
const int TOTAL_BUTTON_INS = 7;
const int BUTTON_INS[] = {37,39,41,43,47,49,51};
const int EYE_RGB_PINS[] = {45,44,46};
const int EYE_RGB_SCALE[] = {120,120,120};
enum EMOTION {UNKNOWN, SAD, HAPPY, ANGRY, SURPRISED, CONFUSED};
const int EXPRESSION_SET[][6] =  {{100,90,80,90,80,4},      //neutral
                                  {150,115,90,70,30,2},     //sad
                                  {20,90,80,90,160,4},      //happy
                                  {80,65,80,115,100,0},     //mad
                                  {20,85,150,95,160,6},     //interested
                                  {20,105,180,75,90,5}};    //uncertain

const int LINACT_APIN[] = {10,0,12,11};
const int LINACT_MAX[] = {800,800,800,800};
const int LINACT_MIN[] = {200,200,200,200};

const int PAN_CENTER = 511;
const int PAN_CENTER_DEV = 154; //90deg*3.4count/deg
const int PAN_MAX = PAN_CENTER + PAN_CENTER_DEV;
const int PAN_MIN = PAN_CENTER - PAN_CENTER_DEV;

const int TORSO_PIN = 53;
const float TORSO_VEL = 2*180/140000; //2mm/s*180deg/140mm*1s/1000ms
const int TORSO_MAXH = 120;
const int TORSO_MINH = 70;
const int TORSO_AVGH = (TORSO_MAXH + TORSO_MINH) / 2;

enum ROBOT_PARTS {WRIST, HEADTILT, GRIPPER_L, GRIPPER_R};

//loop vars
unsigned long prev_time;
unsigned long cur_time;

// button vars
int button_vals[] = {0,0,0,0,0,0,0};
int button_last_vals[] = {0,0,0,0,0,0,0};
int button_active[] = {0,0,0,0,0,0,0};

// torso vars
Servo torso;
int torso_input = 0;
float torso_current = TORSO_AVGH;
float torso_inc = loopTime*TORSO_VEL;
unsigned long prev_torso_time;

// linact vars
AF_DCMotor wrist_motor(1);
AF_DCMotor tilt_motor(2);
AF_DCMotor lh_motor(3);
AF_DCMotor rh_motor(4);

AF_DCMotor linacts[] = {wrist_motor,tilt_motor,lh_motor,rh_motor};

//Define PID Variables
double PIDSet[4], PIDIn[4], PIDOut[4];
double p[] = {1,1,1,1};
double i[] = {1,1,1,1};
double d[] = {0,0,0,0};

PID linactPIDs[] = {
  PID(&PIDIn[0], &PIDOut[0], &PIDSet[0],p[0],i[0],d[0], DIRECT),
  PID(&PIDIn[1], &PIDOut[1], &PIDSet[1],p[1],i[1],d[1], DIRECT),
  PID(&PIDIn[2], &PIDOut[2], &PIDSet[2],p[2],i[2],d[2], DIRECT),
  PID(&PIDIn[3], &PIDOut[3], &PIDSet[3],p[3],i[3],d[3], DIRECT)
};

const long numAvg = 10; //number of running average samples for the linear actuator pot
int indexAvg = 0; //index for running averages

int linact_inputs[] = {0,0,0,0};
int linact_stall_counts[] = {0,0,0,0};
long linact_sensors[4][numAvg];
long linact_sum[4];
int prev_linact_pos[] = {512,512,300,300};

int pan_input = PAN_CENTER;

int currentAct = 0;

// face vars
int lastread;
int face_state = 0;
int face_input = 0;
int expression = 0;
Servo face_servos[TOTAL_FACE_SERVOS];
int human_emo = UNKNOWN;
int dt_face = 0;

void motion_cb(const geometry_msgs::Twist& motion_cmds)
{
  face_input = motion_cmds.linear.x;
  torso_input = motion_cmds.linear.y;
  prev_linact_pos[HEADTILT] = motion_cmds.linear.z;
  pan_input = motion_cmds.angular.x;
  linact_inputs[WRIST] = motion_cmds.angular.y;
  linact_inputs[GRIPPER_L] = motion_cmds.angular.z;
  linact_inputs[GRIPPER_R] = linact_inputs[GRIPPER_L];
}

void emotion_cb(const std_msgs::String& emo_msg)
{
  if (strcmp(emo_msg.data, "HAPPY") == 0) {
    human_emo = HAPPY;
  }
  else if (strcmp(emo_msg.data, "SAD") == 0) {
    human_emo = SAD;
  }
  else if (strcmp(emo_msg.data, "CONFUSED") == 0) {
    human_emo = CONFUSED;
  }
  else if (strcmp(emo_msg.data, "ANGRY") == 0) {
    human_emo = ANGRY;
  }
  else if (strcmp(emo_msg.data, "SURPRISED") == 0) {
    human_emo = SURPRISED;
  }
  else {
    human_emo = UNKNOWN;
  }
}

// Subscribe to ROS topic "/torso_cmds"
ros::Subscriber<geometry_msgs::Twist> sub_motion(
"torso_cmds", &motion_cb);

// Subscribe to ROS topic "/human_emotion"
ros::Subscriber<std_msgs::String> sub_emotion(
"human_emotion", &emotion_cb);

// geometry_msgs::Twist debug_msg;
// ros::Publisher torso_debugger("torso_debugger", &debug_msg);

void linAct(int actuator_num)
{
  int actual_pos;
  int target_pos;

  // increment the target position up or down each cycle
  target_pos = prev_linact_pos[actuator_num] + linact_inputs[actuator_num];
  actual_pos = analogRead(LINACT_APIN[actuator_num]);

  // set limits for input value
  if (target_pos > LINACT_MAX[actuator_num])
  {
    target_pos = LINACT_MAX[actuator_num];
  }
  else if (target_pos < LINACT_MIN[actuator_num])
  {
    target_pos = LINACT_MIN[actuator_num];
  }

  // code for running average
  linact_sum[actuator_num] -=  linact_sensors[actuator_num][indexAvg];
  linact_sensors[actuator_num][indexAvg]=actual_pos;
  linact_sum[actuator_num] +=  actual_pos;

  if (actuator_num == 3)
  {
    indexAvg ++;

    if (indexAvg == numAvg)
    {
      indexAvg = 0;
    }
  }

  PIDSet[actuator_num] = target_pos;
  PIDIn[actuator_num] = linact_sum[actuator_num]/numAvg;
  if ( abs(PIDSet[actuator_num]-PIDIn[actuator_num]) < 5 || (linact_stall_counts[actuator_num] > 40 && linact_inputs[actuator_num] == 0))
  {
    if(actuator_num != HEADTILT){
        linacts[actuator_num].run(RELEASE);
    }
  }
  else
  {

    linactPIDs[actuator_num].Compute();
    linacts[actuator_num].setSpeed(abs(PIDOut[actuator_num]));

    if (PIDOut[actuator_num] < 0)
    {
      linacts[actuator_num].run(FORWARD);
    }
    else
    {
      linacts[actuator_num].run(BACKWARD);
    }
  }

  if (abs(PIDOut[actuator_num]) == 255 && linact_inputs[actuator_num] == 0 )
  {
    linact_stall_counts[actuator_num] ++;
  }
  else
  {
    linact_stall_counts[actuator_num] = 0;
  }

  prev_linact_pos[actuator_num] = target_pos;

}

void setTorsoServo(float increm)
{

  // increment up or down
  torso_current += increm;

  // keep value in range for any increment
  if (torso_current > TORSO_MAXH)
  {
    torso_current = TORSO_MAXH;
  }
  else if (torso_current < TORSO_MINH)
  {
    torso_current = TORSO_MINH;
  }

  torso.write(ceil(torso_current));
}

void moveTorso()
{
  if (torso_input > 0)
  {
    setTorsoServo(0.2);
  }
  else if (torso_input < 0)
  {
    setTorsoServo(-0.2);
  }
}

void panHead()
{
  int target_pos = pan_input;

  // set limits for input value
  if (pan_input > PAN_MAX)
  {
    target_pos = PAN_MAX;
  }
  else if (target_pos < PAN_MIN)
  {
    target_pos = PAN_MIN;
  }

  neckPan.moveJoint(servoID, target_pos);

}

void setEyeColor(int color)
{
  digitalWrite(EYE_RGB_PINS[0], COLORS[color][0]);
  digitalWrite(EYE_RGB_PINS[1], COLORS[color][1]);
  digitalWrite(EYE_RGB_PINS[2], COLORS[color][2]);

  //neckPan.LED(servoID, &RGB[color]);
  neckPan.LED(servoID, &RGB[4]);
}

void setFacialExpression(int exp)
{
  setEyeColor(EXPRESSION_SET[exp][5]);
  face_servos[1].write(EXPRESSION_SET[exp][1]);
  face_servos[3].write(EXPRESSION_SET[exp][3]);
  face_servos[0].write(EXPRESSION_SET[exp][0]);
  face_servos[2].write(EXPRESSION_SET[exp][2]);
  face_servos[4].write(EXPRESSION_SET[exp][4]);
}

void toggleFace()
{
  if(face_input != 0 )
  {
    if(!lastread)
    {
      expression++;
      lastread = 1;

      if(expression == 6)
      {
        expression = 0;
      }
      setFacialExpression(expression);
    }
  }
  else
  {
    lastread = 0;
  }

}

void readButtons()
{
  for (int i = 0; i < TOTAL_BUTTON_INS; i++)
  {
    button_last_vals[i] = button_vals[i];
    button_vals[i]=digitalRead(BUTTON_INS[i]);
    button_active[i]= !button_vals[i] && button_last_vals[i];
  }

  face_input += button_active[6];
  torso_input += (button_vals[5]-button_vals[4]);
  currentAct = (button_active[3]-button_active[2]);

  if(currentAct < 0 )
  {
    currentAct = 4;
  }

  if(currentAct > 4 )
  {
    currentAct = 0;
  }

  if(currentAct ==4)
  {
    pan_input += 5 * (button_vals[1] - button_vals[0]);
  }
  else
  {
    linact_inputs[currentAct] += 5 * (button_vals[1] - button_vals[0]);
  }

}

void setup()
{

  // init neck dynamixel
  Serial1.begin(57600);
  neckPan.begin(Serial1); // Hand in the serial object you're using
  neckPan.setJointSpeed(servoID, neckPanSpeed); // set head pan speed
  neckPan.moveJoint(servoID, PAN_CENTER); // center head position
  // Uncommemt if using the usb2dynamixel for neck panning
  //  pinMode(18, INPUT);
  //  pinMode(19, INPUT);
  // init pin modes
  pinMode(33, INPUT_PULLUP);

  // initialize eye color pins
  for (int i = 0; i < 3; i++)
  {
    pinMode(EYE_RGB_PINS[i], OUTPUT);
  }

  //initialize linact PID
  for (int i = 0; i < 4; i++)
  {
    PIDSet[i] = prev_linact_pos[i];
    PIDIn[i] = analogRead(LINACT_APIN[i]);
    for (int j = 0; j < numAvg; j++)
    {
      linact_sensors[i][j]=PIDIn[i];
    }
    linact_sum[i] = PIDIn[i]*numAvg;

    linactPIDs[i].SetMode(AUTOMATIC);
    linactPIDs[i].SetOutputLimits(-255, 255);
    linactPIDs[i].SetSampleTime(50);
  }

  for (int i = 0; i < TOTAL_BUTTON_INS; i++)
  {
    pinMode(BUTTON_INS[i], INPUT_PULLUP);
  }

  // attach servos to pins
  torso.attach(TORSO_PIN);

  for (int i = 0; i < TOTAL_FACE_SERVOS; i++)
  {
    face_servos[i].attach(FACE_PINS[i]);
  }

  // init timer for loop speed
  prev_time = millis();

  //init timer for torso speed
  prev_torso_time = prev_time;

  // init torso to average height
  torso.write(TORSO_AVGH);

  setFacialExpression(0);

 // init ros stuff
 nh.initNode();
 nh.subscribe(sub_motion);
 nh.subscribe(sub_emotion);

 // nh.advertise(torso_debugger); // comment out when not debugging

}

void loop()
{
  cur_time = millis();

  // ensure minimum time for control loops
  if((cur_time - prev_time) > loopTime)
  {
      prev_time = cur_time;
      // readButtons();

      // mirror or teleop face
      if (human_emo != UNKNOWN) {
        // mirror human expression
        setFacialExpression(human_emo);
        dt_face = cur_time;
      }
      else {
        if (dt_face > 0) {
          // face previously mirrored
          if (dt_face - cur_time > 20000) {
            // set inactive face to neutral after 20s
            setFacialExpression(UNKNOWN);
            dt_face = 0;
          }
        }
        else {
          // check for teleop face cmds
          toggleFace();
        }
      }
      moveTorso();
      panHead();

      linAct(0);
      linAct(1);
      linAct(2);
      linAct(3);
  }

  if((cur_time - prev_neck_time) > neck_set_interval)
  {
    prev_neck_time = cur_time;
    neckPan.setJointSpeed(servoID, neckPanSpeed);
  }

  nh.spinOnce();

  // torso_debugger.publish(&debug_msg); // comment out when not debugging

}
