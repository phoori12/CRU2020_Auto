#include <Arduino.h>

#define feedback_poten 23

#define servo1 22
#define servo2 21
#define servo3 20

#define encA1 33 // back encoder A
#define encB1 34 // back encoder B
#define encB2 35 // left encoder B
#define encA2 36 // left encoder A

#define object_r 37 // object right
#define limit_r 38 // limit right
#define limit_f 39 // limit front
#define sen4 25 // 47
#define sen5 26 // 48
#define sen6 27  // 49
#define limit_l 50 // limit left
#define object_f 51 // object front
#define object_l 52 // object left

#define sw1 47 //  blue sw
#define sw2 48 //  yellow sw
#define sw3 49 //  red sw

//smart drive serial4
//dynamixel ax_18 serial3
//dynamixel mx_18 serial2
// gyro serial1

volatile float gyro_pos = 0;
volatile long ENCL_Count = 0;
volatile long ENCB_Count = 0;

int setpoint = 0;           //////////////////////////////////
float prev_error = 0;       //                              //
float error, p, i, d, edit; //        Heading Tuneup        //
float Kp = 10.0f;           //          Parameters          //
float Ki = 0.00;            //                              //
float Kd = 3.50;            //////////////////////////////////

/////////////// Localization Variables ////////////////////
const float xCon = 0.043422; // To be tuned
const float yCon = 0.039841; // To be tuned
volatile long od1 = 0, od2 = 0;
volatile long last_od1 = 0, last_od2 = 0;
volatile float x_frame, y_frame, x_glob = 0, y_glob = 0;
///////////////////////////////////////////////////////////

/////////////// Function Declaration //////////////////////
void ENCLA_Read();
void ENCLB_Read();
void ENCBA_Read();
void ENCBB_Read();
void getRobotPosition();
void omniControl(int spd, int alpha, int omega);
void headingControl(int spd, int course, int set_head);
void p2ptrack(float set_x, float set_y, float set_head);
    ///////////////////////////////////////////////////////////

    void setup()
{
  // put your setup code here, to run once:
  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);
  pinMode(encA2, INPUT);
  pinMode(encB2, INPUT);
  attachInterrupt(encA1, ENCBA_Read, RISING);
  attachInterrupt(encB1, ENCBB_Read, RISING);
  attachInterrupt(encA2, ENCLA_Read, RISING);
  attachInterrupt(encB2, ENCLB_Read, RISING);
  Serial.begin(9600);
}

void loop() {

}



void ENCBA_Read() {
  if (digitalRead(encB1) == LOW)
  {
    ENCB_Count--;
  }
  else
  {
    ENCB_Count++;
  }
 // Serial.println(ENCL_Count);
}

void ENCBB_Read() {
  if (digitalRead(encA1) == LOW)
  {
    ENCB_Count++;
  }
  else
  {
    ENCB_Count--;
  }
}

void ENCLA_Read() {
  if (digitalRead(encB2) == LOW)
  {
    ENCL_Count++;
  }
  else
  {
    ENCL_Count--;
  }
}

void ENCLB_Read() {
  if (digitalRead(encA2) == LOW)
  {
    //  Serial.println("testttt");
    ENCL_Count--;
  }
  else
  {
    ENCL_Count++;
  }
}