#include <Arduino.h>
#include <Servo.h>

#define feedback_poten 23

Servo servo_right;
Servo servo_left;
Servo servo_mid;
#define servo_r 22
#define servo_l 21
#define servo_m 20

#define encA1 33 // back encoder A
#define encB1 34 // back encoder B
#define encB2 35 // left encoder B
#define encA2 36 // left encoder A

#define object_r 37 // object right
#define limit_r 38 // limit right
#define limit_f 39 // limit front
#define Eobject_m 25 // 47 e_object middle
#define Eobject_r 26 // 48 e_object right // active high // 
#define Eobject_l 27  // 49 e_object left
#define limit_l 50 // limit left
#define object_f 51 // object front
#define object_l 52 // object left

#define sw_blue 47 //  blue sw
#define sw_yell 48 //  yellow sw
#define sw_red 49 //  red sw

#define servo_max 180 // maximum servo span (retraction)
#define servo_med 90  // medium servo span (released)
#define servo_def 0 // defualt servo span (closed)

#define AXFeed_max 550
#define AXFeed_min 0
#define crash_delay 2000
#define gyro_accept 3

const int Direction_RS485 = 11;
//smart drive serial4
//dynamixel ax_18 serial3
//dynamixel mx_18 serial2
// gyro serial1
// via mode
//IntervalTimer rotatorTimer;

//////// Smart Drive /////////
union packed_int
{
  int16_t i;
  byte b[2];
} m1, m2, m3, m4;
//////////////////////////////

#define MAX_SPD 4500
volatile float gyro_pos = 0;
volatile long ENCL_Count = 0;
volatile long ENCB_Count = 0;

int setpoint = 0;           //////////////////////////////////
float prev_error = 0;       //                              //
float error, p, i, d, edit; //        Heading Tuneup        //
float Kp = 30.0f;           //          Parameters          //
float Ki = 0.00;            //                              //
float Kd = 10.5;            //////////////////////////////////

/////////////// Localization Variables ////////////////////
const float xCon = 0.040177f; // To be tuned // 0.043422  // 1/2489
const float yCon = 0.039809f; // To be tuned // 0.039841 // 1/2512
volatile long od1 = 0, od2 = 0;
volatile long last_od1 = 0, last_od2 = 0;
volatile float x_frame, y_frame, x_glob = 0, y_glob = 0;
///////////////////////////////////////////////////////////

//////////////////// Crash Detection ///////////////////////

bool crash_status = false;
uint16_t crash_time = 0;
uint16_t feedingTime = 0;
bool rotate_rdy =false;

////////////////////////////////////////////////////////////
///////////////////// P2P Control Vars /////////////////////
float mapgyro;
float d_i = 0;
const float s_kp = 77.0f, s_ki = 0.50f, s_kd = 24.0f;
const float h_kp = 30.0f, h_ki = 0.09f, h_kd = 10.50f;
float dx, dy, dsm, s_error, d_s, s_edit, compensateTht;
float h_edit, h_error = 0, h_preverror = 0, h_i = 0, h_d = 0;
long targetTime = 0;
////////////////////////////////////////////////////////////

//////////////////////// Rotator PID ////////////////////////
int rotator_leftpos = 815;
int rotator_midpos = 858;
int rotator_rightpos = 904; // 900
int rotator_maxSpeed = 400;

float r_Kp = 10.0f, r_Kd = 2;
float r_prev_error = 0;
float r_error, r_p, r_d, r_edit;
long r_ontarget;
bool r_atTarget = false;

////////////////////////////////////////////////////////////////

volatile int rotatorPosition = rotator_midpos, feederPosition = AXFeed_min;
volatile bool rotateFlag = false;
uint8_t AddressGoalPosition = 30;
uint8_t AddressMovingSpeed = 0x20;
uint8_t cmdWrite = 3;
uint8_t Dynamixel_feed = 1;
uint8_t Dynamixel_rotate = 1;
uint8_t gyro_offset = 3;

/////////////// Function Declaration //////////////////////
float degToRad(int val);
void ENCLA_Read();
void ENCLB_Read();
void ENCBA_Read();
void ENCBB_Read();
void getRobotPosition();
void omniControl(int spd, int alpha, int omega);
void headingControl(int spd, int course, int set_head);
void p2ptrack(float set_x, float set_y, float set_head, bool viaMode, int rPos);
void sendDriveCmd(int spd1, int spd2, int spd3, int spd4);
void stopCmd();
void writeToFeeder(uint16_t data, uint8_t id, uint8_t instruction, uint8_t Address);
void writeToRotator(int16_t speed, uint8_t id, uint8_t instruction, uint8_t Address);
void rotatorControl(int position, int speed);
void rotatorInterval();
void setFeeder(int feederpos);
// void feederInterval();
///////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
  Serial1.begin(115200); // Gyro Serial Init
  Serial2.begin(57600);  // Dynamixel MX18
  Serial3.begin(57600);  // Dynamixel AX18
  Serial4.begin(115200); // Smart Drive Serial Init
  pinMode(Direction_RS485, OUTPUT);
  digitalWrite(Direction_RS485, HIGH); //send value
  /// Servo Init ///
  servo_right.attach(servo_r);
  servo_left.attach(servo_l);
  servo_mid.attach(servo_m);
  servo_left.write(0);
  servo_right.write(0);
  servo_mid.write(0);
  delay(1000);
  pinMode(13, INPUT);
  digitalWrite(13, LOW);
  //////// Switch Init ////////
  pinMode(sw_blue, INPUT); // Plan 1
  pinMode(sw_yell, INPUT); // Plan 2
  pinMode(sw_red, INPUT);  // Plan 3
  //////// Bottom sensor init ////////
  pinMode(feedback_poten, INPUT); // analog potentiometer
  pinMode(object_f, INPUT);
  pinMode(limit_f, INPUT);
  pinMode(object_l, INPUT);
  pinMode(limit_l, INPUT);
  pinMode(object_r, INPUT);
  pinMode(limit_r, INPUT);
  pinMode(Eobject_l, INPUT);
  pinMode(Eobject_m, INPUT);
  pinMode(Eobject_r, INPUT);
  //////// Encoder Init ////////
  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);
  pinMode(encA2, INPUT);
  pinMode(encB2, INPUT);
  attachInterrupt(encA1, ENCBA_Read, RISING);
  attachInterrupt(encB1, ENCBB_Read, RISING);
  attachInterrupt(encA2, ENCLA_Read, RISING);
  attachInterrupt(encB2, ENCLB_Read, RISING);
  //////// Gyro reset ////////
  Serial1.write(0xA5); // set Gyro heading to 0
  Serial1.write(0x55);
  delay(2000);
  Serial1.write(0xA5);
  Serial1.write(0x52); // set Gyro Read mode 2
  delay(2000);
  digitalWrite(13, HIGH);
  delay(100);
  /////////////////////////////
  /////////////// Set 0 Dynamixel ///////////////
  rotatorControl(rotator_midpos, rotator_maxSpeed);
  writeToFeeder(AXFeed_min, Dynamixel_feed, cmdWrite, AddressGoalPosition);
  delay(500);
  //////////////////////////////////////////////
 // rotatorTimer.begin(rotatorInterval, 10000);
}


// 180 - > right
// 0 -> left
// 90 -> front
// 270 -> bacl


// ลบหมุนขวา บวกหมุนซ้าย
void loop() {
    feedingTime = 0;
    p2ptrack(13, -62, 0, true, rotator_midpos); //x 16 y -62
    stopCmd();
    delay(50);

    feedingTime = millis();
    rotateFlag  = false;
    writeToFeeder(AXFeed_max - 50, Dynamixel_feed, cmdWrite, AddressGoalPosition);
    delay(200);
    // rotatorControl(rotator_rightpos, rotator_maxSpeed);
    // delay(500);
    // p2ptrack(25, -62, 0); //x 25 y -62
    // stopCmd();
    // delay(100);

    while (digitalRead(limit_r))
    {
      getRobotPosition();
      headingControl(1500, 180, 0);
      if (od1 == 0 && !crash_status)
      { // od2 for y axis
        crash_time = millis();
        crash_status = true;
      }

      if (millis() - crash_time > crash_delay && crash_status)
      {
        crash_time = 0;
        crash_status = false;
        break;
      }
    }
    rotateFlag = false;
    // stopCmd();
    // delay(100);
    // //rotatorControl(rotator_rightpos, rotator_maxSpeed);
    // servo_right.write(servo_med);
    // delay(500);
    // servo_right.write(servo_max);
    // delay(500);
    // /////////////////////////////////////////////////////////////////
    // p2ptrack(4, -62, 0, true); //x 16 y -62
    // stopCmd();
    // delay(100);
    // // servo_right.write(servo_def);
    // // delay(500);
    // // p2ptrack(13, -102, 0, true); //x 16 y -62
    // // stopCmd();
    // // delay(50);
    // // p2ptrack(58, -102, 0); //x 16 y -62
    // // stopCmd();
    // // delay(100);
    // p2ptrack(58, -115, 0); //x 16 y -62
    // stopCmd();
    // delay(50);
    
    // while (digitalRead(limit_f))
    // {
    //   getRobotPosition();
    //   headingControl(1500, 90, 0);
    //   if (od2 == 0 && !crash_status) { // od2 for y axis
    //     crash_time = millis();
    //     crash_status = true;
    //   } else {
    //     crash_status = false;
    //   }

    //   if (millis() - crash_time > crash_delay && crash_status) {
    //     crash_time = 0;
    //     crash_status = false;
    //     break;
    //   }

    // }
    // stopCmd();
    // delay(100);
    // while (digitalRead(limit_f))
    // {
    //   getRobotPosition();
    //   headingControl(1500, 90, 0);
    //   if (od2 == 0 && !crash_status) { // od2 for y axis
    //     crash_time = millis();
    //     crash_status = true;
    //   } else {
    //     crash_status = false;
    //   }

    //   if (millis() - crash_time > crash_delay && crash_status) {
    //     crash_time = 0;
    //     crash_status = false;
    //     break;
    //   }

    // }
    // stopCmd();
    // delay(100);
    // // writeToFeeder(AXFeed_max, Dynamixel_feed, cmdWrite, AddressGoalPosition);
    // // delay(500);
    // // rotatorControl(rotator_midpos, rotator_maxSpeed);
    // // servo_mid.write(servo_med);
    // // delay(500);
    // // servo_mid.write(servo_max);
    // // delay(500);
    // // /////////////////////////////////////////////////////////////////////////////////
    // p2ptrack(62, -120, 0, true); //x 16 y -62
    // stopCmd();
    // delay(10);
    // // servo_mid.write(servo_def);
    // // delay(500);
    // // p2ptrack(105, -110, 0, true); //x 16 y -62
    // // stopCmd();
    // // delay(50);
    // p2ptrack(110, -65, 0); //x 16 y -62
    // stopCmd();
    // delay(100);
    // // p2ptrack(95, -65, 0); //x 16 y -62
    // // stopCmd();
    // // delay(100);
    // while (digitalRead(limit_l))
    // {
    //   getRobotPosition();
    //   headingControl(1500, 0, 0);
    //   if (od1 == 0 && !crash_status)
    //   { // od2 for y axis
    //     crash_time = millis();
    //     crash_status = true;
    //   }

    //   if (millis() - crash_time > crash_delay && crash_status)
    //   {
    //     crash_time = 0;
    //     crash_status = false;
    //     break;
    //   }
    // }
    // stopCmd();
    // delay(100);

    // while (digitalRead(object_r))
    // {
    //   getRobotPosition();
    //   headingControl(1500, 180, 0);
    //   if (od1 == 0 && !crash_status)
    //   { // od2 for y axis
    //     crash_time = millis();
    //     crash_status = true;
    //   }

    //   if (millis() - crash_time > crash_delay && crash_status)
    //   {
    //     crash_time = 0;
    //     crash_status = false;
    //     break;
    //   }
    // }
    // stopCmd();
    // delay(100);

    // while (digitalRead(object_f))
    // {
    //   getRobotPosition();
    //   headingControl(1500, 100, 0);
    //   if (od2 == 0 && !crash_status)
    //   { // od2 for y axis
    //     crash_time = millis();
    //     crash_status = true;
    //   }
    //   else
    //   {
    //     crash_status = false;
    //   }

    //   if (millis() - crash_time > crash_delay && crash_status)
    //   {
    //     crash_time = 0;
    //     crash_status = false;
    //     break;
    //   }
    // }
    // stopCmd();
    // delay(100);
    // writeToFeeder(AXFeed_max - 50, Dynamixel_feed, cmdWrite, AddressGoalPosition);
    // delay(500);
    // rotatorControl(rotator_leftpos, rotator_maxSpeed);
    // servo_left.write(servo_med);
    // delay(500);
    // servo_left.write(servo_max);
    // delay(500);
    // p2ptrack(122, -65, 0); //x 16 y -62
    // stopCmd();
    // delay(100);
    // servo_left.write(servo_def);
    // delay(500);
    // // p2ptrack(122, -5, 0); //x 16 y -62
    // // stopCmd();
    // // delay(100);
    // p2ptrack(122, -5, 0); //x 16 y -62
    // stopCmd();
    // delay(100);
    // rotatorControl(rotator_midpos, rotator_maxSpeed);
    // writeToFeeder(AXFeed_min, Dynamixel_feed, cmdWrite, AddressGoalPosition);
    // delay(500);
    while (1)
    {
      stopCmd();
      getRobotPosition();
      Serial.print(x_glob);
      Serial.print("\t");
      Serial.println(y_glob);
      //stopCmd();
    }
  }

// void rotatorInterval() 
// {
//   if (rotateFlag == true) {
//     rotatorControl(rotatorPosition, rotator_maxSpeed);
//   } else {
//     writeToRotator(0, Dynamixel_rotate, cmdWrite, AddressMovingSpeed);
//   }
  
// }

  void stopCmd()
  {
    sendDriveCmd(0, 0, 0, 0);
  }

  void getRobotPosition()
  {
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
    // Serial.print(gyro_pos);
    // Serial.print("\t");
    // Serial.print(x_glob);
    // Serial.print("\t");
    // Serial.println(y_glob);
  }

  void omniControl(int spd, int alpha, int omega)
  {
    int w1, w2, w3, w4;
    w1 = spd * cos(degToRad(135) - degToRad(alpha)) + omega; // 315
    w2 = spd * cos(degToRad(225) - degToRad(alpha)) + omega; // 45
    w3 = spd * cos(degToRad(315) - degToRad(alpha)) + omega; // 135
    w4 = spd * cos(degToRad(45) - degToRad(alpha)) + omega;  // 225

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

  void headingControl(int spd, int course, int set_head)
  {
    if (abs(gyro_pos) - set_head > gyro_accept)
    {
      error = gyro_pos - set_head;
    }
    else
    {
      error = 0;
    }
    p = Kp * error;
    d = (error - prev_error) * Kd;
    prev_error = error;
    edit = p + d;
    omniControl(spd, course, edit);
  }

  void p2ptrack(float set_x, float set_y, float set_head, bool viaMode, int rPos)
  {

    static volatile float s_prev_error = 0.0f;
    static bool atTarget = false;
    static float theta = 0;
    static uint32_t last_time = 0;

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

      if ((abs(dx) <= 3 && abs(dy) <= 3) && abs(h_error) <= gyro_accept)
      {

        if (atTarget == false)
        {
          targetTime = millis();
        }
        atTarget = true;
        if (millis() - targetTime > 500 || viaMode)
        {
          // atTarget = true;
          sendDriveCmd(0, 0, 0, 0);
          break;
        }
      }
      else
      {
        atTarget = false;
      }

      omniControl(s_edit, compensateTht, h_edit);
      if (millis() - last_time >= 100)
      {
          last_time = millis();
          if (millis() - feedingTime > 300)
          {
            rotatorControl(rPos, rotator_maxSpeed);
          } else {
          writeToRotator(0, Dynamixel_rotate, cmdWrite, AddressMovingSpeed);
          }
      }
      // headingControl(s_edit, compensateTht, set_head);
      }
  }

  void writeToFeeder(uint16_t data, uint8_t id, uint8_t instruction, uint8_t Address)
  {
    byte dataSend[9];
    dataSend[0] = 0xFF;
    dataSend[1] = 0xFF;
    dataSend[2] = id;
    dataSend[3] = 0x05; // length
    dataSend[4] = (byte)instruction;
    dataSend[5] = (byte)Address;
    dataSend[6] = (byte)(data & 0xFF);
    dataSend[7] = (byte)(data >> 8);

    uint16_t checksum = ~(id + dataSend[3] + dataSend[4] + dataSend[5] + dataSend[6] + dataSend[7]);
    dataSend[8] = (checksum & 0xFF);

    for (int i = 0; i <= 9; i++)
    {
      Serial3.write(dataSend[i]);
      delay(1);
    }
    delay(50);
  }

  void writeToRotator(int16_t speed, uint8_t id, uint8_t instruction, uint8_t Address)
  {
    uint8_t checksum;
    uint8_t serial_send[9];

    if (speed < 0)
    {
      speed = abs(speed) + 1023;
    }

    serial_send[0] = 0xFF;
    serial_send[1] = 0xFF;
    serial_send[2] = id;
    serial_send[3] = 0x05; // 0x05
    serial_send[4] = (byte)instruction; // 0x03
    serial_send[5] = (byte)Address;     // 0x20
    serial_send[6] = (byte)(speed & 0xFF);
    serial_send[7] = (byte)(speed >> 8);
    checksum = (serial_send[2] + serial_send[3] + serial_send[4] + serial_send[5] + serial_send[6] + serial_send[7]);
    serial_send[8] = ~(checksum & 0xFF);
    for (uint8_t i = 0; i < 9; i++)
    {
      Serial2.write(serial_send[i]);
      delay(10);
    }
  }

  void sendDriveCmd(int spd1, int spd2, int spd3, int spd4)
  {
    m1.i = spd1;
    m2.i = spd2;
    m3.i = spd3;
    m4.i = spd4;
    const char cmd[12] = {'#', 's', m1.b[1], m1.b[0],
                          m2.b[1], m2.b[0], m3.b[1], m3.b[0], m4.b[1], m4.b[0],
                          '\r', '\n'};
    for (uint8_t i = 0; i < 12; i++)
    {
      // Serial.write(cmd[i]);
      Serial4.write(cmd[i]);
    }
  }

  void rotatorControl(int position, int speed)
  {
    while (1) {
      int feedback = analogRead(feedback_poten);
      // Serial.println(feedback- position);
      if (abs(feedback - position) > 4)
      {
        r_error = feedback - position;
      }
      else
      {
        r_error = 0;
        r_prev_error = 0;
      }
      r_p = r_Kp * r_error;
      r_d = (r_error - r_prev_error) * r_Kd;
      r_prev_error = r_error;
      r_edit = r_p + r_d;
      if (r_edit > rotator_maxSpeed)
        r_edit = rotator_maxSpeed;
      if (-r_edit < -rotator_maxSpeed)
        r_edit = -rotator_maxSpeed;
      if (r_edit == 0 && r_atTarget == false)
      {
        r_ontarget = millis();
        r_atTarget = true;
      }
      else if (r_edit != 0)
      {
        r_atTarget = false;
      }
      if (millis() - r_ontarget > 500 && r_atTarget)
      {
        // atTarget = true;
        writeToRotator(0, Dynamixel_rotate, cmdWrite, AddressMovingSpeed);
        r_atTarget = false;
        break;
      }
      writeToRotator(r_edit, Dynamixel_rotate, cmdWrite, AddressMovingSpeed);
    }
    
    // Serial.println(r_edit);
   
    
  }

  void serialEvent1()
  {
    while (Serial1.available())
    {
      static unsigned char Buffer_gyro[8] = {};
      static unsigned char count_gyro = 0;
      volatile unsigned char Buffer = Serial1.read();
      Buffer_gyro[count_gyro] = Buffer;
      if (count_gyro == 0 && Buffer_gyro[0] != 0xAA)
        return;
      count_gyro++;

      if (count_gyro == 8)
      {
        count_gyro = 0;

        if (Buffer_gyro[0] == 0xAA && Buffer_gyro[7] == 0x55)
        {
          //robot.smooth((int16_t)(Buffer_gyro[1] << 8 | Buffer_gyro[2]) / 100.00, 0.95, robot.gyro_pos);
          gyro_pos = (int16_t)(Buffer_gyro[1] << 8 | Buffer_gyro[2]) / 100.00;
        }
      }
    }
  }

  float degToRad(int val)
  {
    return val * (PI / 180);
  }

  void ENCBA_Read()
  {
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

  void ENCBB_Read()
  {
    if (digitalRead(encA1) == LOW)
    {
      ENCB_Count++;
    }
    else
    {
      ENCB_Count--;
    }
  }

  void ENCLA_Read()
  {
    if (digitalRead(encB2) == LOW)
    {
      ENCL_Count++;
    }
    else
    {
      ENCL_Count--;
    }
  }

  void ENCLB_Read()
  {
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