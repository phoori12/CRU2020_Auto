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

//////// Smart Drive /////////
union packed_int
{
  int16_t i;
  byte b[2];
} m1, m2, m3, m4;
//////////////////////////////

#define MAX_SPD 2500
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

///////////////////// P2P Control Vars /////////////////////
float mapgyro;
float d_i = 0;
const float s_kp = 30.0f, s_ki = 0.25f, s_kd = 8.0f;
const float h_kp = 10.0f, h_ki = 0.03f, h_kd = 3.50f;
float dx, dy, dsm, s_error, d_s, s_edit, compensateTht;
float h_edit, h_error = 0, h_preverror = 0, h_i = 0, h_d = 0;
long targetTime = 0;
////////////////////////////////////////////////////////////

/////////////// Function Declaration //////////////////////
float degToRad(int val);
void ENCLA_Read();
void ENCLB_Read();
void ENCBA_Read();
void ENCBB_Read();
void getRobotPosition();
void omniControl(int spd, int alpha, int omega);
void headingControl(int spd, int course, int set_head);
void p2ptrack(float set_x, float set_y, float set_head);
void sendDriveCmd(int spd1, int spd2, int spd3, int spd4);
///////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
  Serial4.begin(115200); // Smart Drive Serial Init
  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);
  pinMode(encA2, INPUT);
  pinMode(encB2, INPUT);
  attachInterrupt(encA1, ENCBA_Read, RISING);
  attachInterrupt(encB1, ENCBB_Read, RISING);
  attachInterrupt(encA2, ENCLA_Read, RISING);
  attachInterrupt(encB2, ENCLB_Read, RISING);

}

void loop() {

}

void getRobotPosition() {
  static long od1_now = 0, od2_now = 0;
  od1_now = ENCB_Count;
  od2_now = ENCL_Count;

  od1 = od1_now - last_od1;
  od2 = od2_now - last_od2;
  last_od1 = od1_now;
  last_od2 = od2_now;
  x_frame = od1 * xCon;
  y_frame = od2 * yCon;

  x_glob += (x_frame * cos(degToRad(gyro_pos)) - y_frame * sin(degToRad(gyro_pos)));
  y_glob += (x_frame * sin(degToRad(gyro_pos)) + y_frame * cos(degToRad(gyro_pos)));
  Serial.print(gyro_pos);
  Serial.print("\t");
  Serial.print(x_glob);
  Serial.print("\t");
  Serial.println(y_glob);
}

void omniControl(int spd, int alpha, int omega) {
  int w1, w2, w3, w4;
  w1 = spd * cos(degToRad(315) - degToRad(alpha)) + omega;
  w2 = spd * cos(degToRad(45) - degToRad(alpha)) + omega;
  w3 = spd * cos(degToRad(135) - degToRad(alpha)) + omega;
  w4 = spd * cos(degToRad(225) - degToRad(alpha)) + omega;

  if (w1 > MAX_SPD)
    w1 = MAX_SPD;
  if (w1 < -MAX_SPD)
    w1 = -MAX_SPD;
  if (w2 > MAX_SPD)
    w2 = MAX_SPD;
  if (w2 < -MAX_SPD)
    w2 = -MAX_SPD;
  if (w3 > MAX_SPD)
    w3 = MAX_SPD;
  if (w3 < -MAX_SPD)
    w3 = -MAX_SPD;
  if (w4 > MAX_SPD)
    w4 = MAX_SPD;
  if (w4 < -MAX_SPD)
    w4 = -MAX_SPD;
  // Serial.print(w1);
  // Serial.print("\t");
  // Serial.print(w2);
  // Serial.print("\t");
  // Serial.print(w3);
  // Serial.print("\t");
  // Serial.println(w4);
  sendDriveCmd(w1, w2, w3, w4);
}

void headingControl(int spd, int course, int set_head) {
  if (abs(gyro_pos) - set_head > 2)
  {
    error = gyro_pos - set_head;
    p = Kp * error;
    d = (error - prev_error) * Kd;
    prev_error = error;
  }
  else
  {
    error = 0;
    prev_error = 0;
  }
  edit = p + d;
  omniControl(spd, course, edit);
}

void p2ptrack(float set_x, float set_y, float set_head) {

  static volatile float s_prev_error = 0.0f;
  static bool atTarget = false;
  static float theta = 0;
  while (1)
  {

    getRobotPosition();
    dx = set_x - x_glob;
    dy = set_y - y_glob;
    dsm = sqrt(pow(dx, 2) + pow(dy, 2));

    s_error = dsm;
    d_i += s_error;
    d_i = constrain(d_i, 0, 1000);
    d_s = s_error - s_prev_error;
    s_prev_error = s_error;
    h_error = gyro_pos - set_head;
    h_i += h_error;
    h_i = constrain(h_i, 0, 1000);
    h_d = h_error - h_preverror;
    h_preverror = h_error;

    if (dy == 0)
    {
      if (dx < 0)
      {
        theta = 0;
      }
      else if (dx > 0)
      {
        theta = 180;
      }
    }
    else if (dx == 0)
    {
      if (dy < 0)
      {
        theta = 270;
      }
      else if (dy > 0)
      {
        theta = 90;
      }
    }
    else
    {
      theta = atan2(dy, dx) * (180 / PI);
      theta = 180 - theta;
    }
    serialEvent1();
    mapgyro = gyro_pos;
    if (mapgyro > 0)
    {
      mapgyro = fmod(mapgyro, 360);
    }
    else if (mapgyro < 0)
    {
      mapgyro = fmod(abs(mapgyro), 360 * -1);
    }

    s_edit = (s_error * s_kp) + (d_i * s_ki) + (s_kd * d_s);
    h_edit = (h_error * h_kp) + (h_i * h_ki) + (h_kd * h_d);
    compensateTht = theta + mapgyro;

    if ((abs(dx) <= 3 && abs(dy) <= 3) && abs(h_error) <= 5)
    {
      if (atTarget == false)
      {
        targetTime = millis();
      }
      atTarget = true;
      if (millis() - targetTime > 500)
      {
        // atTarget = true;
        sendDriveCmd(0,0,0,0);
        break;
      }
    }
    else
    {
      atTarget = false;
    }

    omniControl(s_edit, compensateTht, h_edit);

    // headingControl(s_edit, compensateTht, set_head);
  }
}

void sendDriveCmd(int spd1, int spd2, int spd3, int spd4) {
  m1.i = spd1;
  m2.i = spd2;
  m3.i = -spd3;
  m4.i = -spd4;
  const char cmd[12] = {'#', 's', m1.b[1], m1.b[0],
                        m2.b[1], m2.b[0], m3.b[1], m3.b[0], m4.b[1], m4.b[0],
                        '\r', '\n'};
  for (uint8_t i = 0; i < 12; i++)
  {
    // Serial.write(cmd[i]);
    Serial4.write(cmd[i]);
  }
}

float degToRad(int val)
{
  return val * (PI / 180);
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