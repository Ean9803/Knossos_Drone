
#include <Wire.h>
#include <Servo.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.20;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.02;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 13.00;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;
float pid_i_gain_pitch = pid_i_gain_roll;
float pid_d_gain_pitch = pid_d_gain_roll;
int pid_max_pitch = pid_max_roll;

float pid_p_gain_yaw = 3.00;
float pid_i_gain_yaw = 0.02;
float pid_d_gain_yaw = 0.00;
int pid_max_yaw = 400;

//Declaring some global variables
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, timer_channel_5, timer_channel_6, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
int throttle, battery_voltage;
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4, esc_5, esc_6;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
int start, lastStart;

Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;
Servo ESC5;
Servo ESC6;

float InputX, InputY, InputZ, InputW;
int update_interval=100; // time interval in ms for updating panel indicators 
unsigned long last_time=0; // time of last update
char data_in; // data received from serial link
String terminal_str=""; // String for Terminal Send Boxes
int progress; // Progress Bar Value

String Outputs[5];
String CmdString = "";

void arm()
{
  setSpeed(ESC1, 0);
  setSpeed(ESC2, 0);
  setSpeed(ESC3, 0);
  setSpeed(ESC4, 0);
  setSpeed(ESC5, 0);
  setSpeed(ESC6, 0);
}

void setSpeed(Servo ESC, int speed)
{
  int angle = map(speed, 1000, 2000, 30, 180); //Sets servo positions to different speeds
  ESC.write(angle);
}

void setOutput(int Index, String Value)
{
  if (Index < 0 && Index >= 5)
  {
    Index = 0;
  }
  Outputs[Index] = Value;
}


int lines = 0;
void printStr(String Value)
{
  if (lines++ > 12)
  {
    CmdString = CmdString.substring(CmdString.indexOf('\n') + 1);
    lines--;
  }
  CmdString.concat(Value);
  CmdString.concat("\n");

  Serial.print("*&"+CmdString+"*");
}

void setup()
{

  Serial.begin(9600);

  Serial.println("*.kwl");
  Serial.println("clear_panel()");
  Serial.println("set_grid_size(30,14)");
  Serial.println("add_text_box(13,7,8,L,,245,240,245,4)");
  Serial.println("add_text_box(9,7,3,L,[4]:,245,240,245,)");
  Serial.println("add_text_box(13,5,8,L,,245,240,245,2)");
  Serial.println("add_text_box(13,6,8,L,,245,240,245,3)");
  Serial.println("add_text_box(13,4,8,L,,245,240,245,1)");
  Serial.println("add_text_box(13,3,8,L,,245,240,245,0)");
  Serial.println("add_text_box(9,3,3,L,[0]:,245,240,245,)");
  Serial.println("add_text_box(9,4,3,L,[1]:,245,240,245,)");
  Serial.println("add_text_box(9,6,3,L,[3]:,245,240,245,)");
  Serial.println("add_text_box(9,5,3,L,[2]:,245,240,245,)");
  Serial.println("add_switch(14,0,4,S,K,0,0)");
  Serial.println("add_touch_pad(2,5,6,-100,100,0,100,A,a)");
  Serial.println("add_touch_pad(22,5,6,-100,100,0,100,B,b)");
  Serial.println("add_gauge(2,11,4,0,200,100,Y,,,10,5)");
  Serial.println("add_gauge(22,11,4,0,200,100,R,,,10,5)");
  Serial.println("add_gauge(1,5,3,0,200,0,T,,,10,5)");
  Serial.println("add_gauge(28,5,3,0,200,100,P,,,10,5)");
  Serial.println("add_gauge(12,2,5,0,100,100,?,,,10,5)");
  Serial.println("add_send_box(9,9,5,,@,;)");
  Serial.println("add_monitor(15,8,6,&,3)");
  Serial.println("set_panel_notes(DroneRemote,,,)");
  Serial.println("run()");

  ESC1.attach(4); //Adds ESC to certain pin.
  ESC2.attach(5); //Adds ESC to certain pin.
  ESC3.attach(6); //Adds ESC to certain pin.
  ESC4.attach(7); //Adds ESC to certain pin.
  ESC5.attach(8); //Adds ESC to certain pin.
  ESC6.attach(9); //Adds ESC to certain pin.
  arm();

  Wire.begin();                                                        //Start I2C as master

  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  printStr("Booting Up");
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {
    if (cal_int % 125 == 0) {
      printStr("-");
    }
    read_mpu_6050_data();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    delayMicroseconds(1000);
    delay(3);   //Delay 3us to simulate the 25
  }

  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;

  delay(50);
  start = 0;
  loop_timer = micros();

  printStr("Ready");

  InputX = InputY = InputZ = InputW = 0;
  lastStart = -1;
}

long readVcc()
{
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result;
}

void loop()
{
  BluetoothInput();

  read_mpu_6050_data();

  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = 0;//(gyro_roll_input * 0.8) + ((gyro_x / 57.14286) * 0.2);            //Gyro pid input is deg/sec.
  gyro_pitch_input = 0;//(gyro_pitch_input * 0.8) + ((gyro_y / 57.14286) * 0.2);         //Gyro pid input is deg/sec.
  gyro_yaw_input = 0;//(gyro_yaw_input * 0.8) + ((gyro_z / 57.14286) * 0.2);               //Gyro pid input is deg/sec.

  calculate_angle();

/*
  if (start == 0 && receiver_input_channel_3 < 1100 && receiver_input_channel_4 < 1100){
    start = 1;
  }
  if (start == 1 && receiver_input_channel_3 < 1100 && receiver_input_channel_4 > 1450){
    resetPID();
    Serial.println("Motors Started");
    delay(50);
    start = 2;
  }
  if (start == 2 && receiver_input_channel_3 < 1100 && receiver_input_channel_4 > 1900){
    start = 0;
    Serial.println("Stopeed");
    delay(40);
  }
*/
  //channel 1 --> yaw
  //channel 2 --> throttle
  //channel 3 --> pitch
  //channel 4 --> roll
/*
  receiver_input_channel_1 = map(analogRead(A0), 0, 1023, 1000, 2000);
  receiver_input_channel_2 = map(analogRead(A1), 0, 1023, 1000, 2000);
  receiver_input_channel_3 = map(analogRead(A2), 0, 1023, 1000, 2000);
  receiver_input_channel_4 = map(analogRead(A3), 0, 1023, 1000, 2000);
*/

  pid_roll_setpoint = 0;
  if (receiver_input_channel_4 > 1510) pid_roll_setpoint = receiver_input_channel_4 - 1510;
  else if (receiver_input_channel_4 < 1490) pid_roll_setpoint = receiver_input_channel_4 - 1490;

  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

  pid_pitch_setpoint = 0;
  if (receiver_input_channel_3 > 1510) pid_pitch_setpoint = receiver_input_channel_3 - 1510;
  else if (receiver_input_channel_3 < 1490) pid_pitch_setpoint = receiver_input_channel_3 - 1490;

  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  pid_yaw_setpoint = 0;
  if (receiver_input_channel_2 > 1080) { //Do not yaw when turning off the motors.
    if (receiver_input_channel_1 > 1510) pid_yaw_setpoint = (receiver_input_channel_1 - 1510) / 5.0;
    else if (receiver_input_channel_1 < 1490) pid_yaw_setpoint = (receiver_input_channel_1 - 1490) / 5.0;
  }

  //---------------------------------------------------------------------------------------
  //calculate_pid();
  pid_output_yaw = map(InputW, -100, 100, -180, 180);
  receiver_input_channel_2 = map(InputZ, 100, -100, 1000, 2000);
  pid_output_pitch = map(InputY, 100, -100, -180, 180);
  pid_output_roll = map(InputX, -100, 100, -180, 180);
  //---------------------------------------------------------------------------------------

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  battery_voltage = (int)readVcc();

  throttle = receiver_input_channel_2;

  if (start != lastStart)
  {
    if (start == 0)
      printStr("KILLED");
    else if (start == 2)
      printStr("STARTED");
    lastStart = start;
  }

  if (start == 1)
  {
    printStr("STAND_BY -> " + String(-InputZ) + " <= -90");
    if (-InputZ <= -90)
      start = 2;
  }

  if (start == 2) {

    if (throttle > 1800) throttle = 1800;

    esc_2 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //pulse for esc 1 (front-right - CCW)
    esc_1 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //pulse for esc 2 (rear-right - CW)
    esc_4 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //pulse for esc 3 (rear-left - CCW)
    esc_3 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //pulse for esc 4 (front-left - CW)
    esc_5 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //?
    esc_6 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //?

    //voltage drop calculation
    if (battery_voltage < 1240 && battery_voltage > 800 && false) {
      esc_1 += esc_1 * ((1240 - battery_voltage) / (float)3500);
      esc_2 += esc_2 * ((1240 - battery_voltage) / (float)3500);
      esc_3 += esc_3 * ((1240 - battery_voltage) / (float)3500);
      esc_4 += esc_4 * ((1240 - battery_voltage) / (float)3500);
      esc_5 += esc_5 * ((1240 - battery_voltage) / (float)3500);
      esc_6 += esc_6 * ((1240 - battery_voltage) / (float)3500);
    }

    if (esc_1 < 1100) esc_1 = 1000;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1000;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1000;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1000;                                      //Keep the motors running.
    if (esc_5 < 1100) esc_5 = 1000;                                         //Keep the motors running.
    if (esc_6 < 1100) esc_6 = 1000; 

    if (esc_1 > 2000) esc_1 = 2000;                                          //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000) esc_2 = 2000;                                          //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000) esc_3 = 2000;                                          //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000) esc_4 = 2000;
    if (esc_5 > 2000) esc_5 = 2000;
    if (esc_6 > 2000) esc_6 = 2000;
  }
  else {
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
    esc_5 = 1000;
    esc_6 = 1000;
  }
/*
  Serial.print("ESC_1:");
  Serial.print(esc_1);
  Serial.print(",");
  Serial.print("ESC_2:");
  Serial.print(esc_2);
  Serial.print(",");
  Serial.print("ESC_3:");
  Serial.print(esc_3);
  Serial.print(",");
  Serial.print("ESC_4:");
  Serial.print(esc_4);
  Serial.print(",");
  Serial.print("ESC_5:");
  Serial.print(esc_5);
  Serial.print(",");
  Serial.print("ESC_6:");
  Serial.print(esc_6);
  Serial.print(",");
  Serial.print("BAT:");
  Serial.println(battery_voltage);
  */
  setSpeed(ESC1, esc_1);
  setSpeed(ESC2, esc_2);
  setSpeed(ESC3, esc_3);
  setSpeed(ESC4, esc_4);
  setSpeed(ESC5, esc_5);
  setSpeed(ESC6, esc_6);
/*
  printStr("Throttle: " + String(InputZ, 3));
  printStr("Yaw: " + String(InputW, 3));
  printStr("Pitch: " + String(InputY, 3));
  printStr("Roll: " + String(InputX, 3));
*/
}

void BluetoothInput()
{
   if (Serial.available())
   {
    data_in=Serial.read(); //Get next character 

    if(data_in=='S')
    { //Switch On
      resetPID();
      start = 1;
    }
    if(data_in=='K')
    { // Switch Off 
      start = 0;
    }
    if(data_in=='F')
    { // Force ON 
      start = 2;
    }

    //Touch pad
    if(data_in=='B')
    { // Pad Start Text
      while(true)
      {
        if (Serial.available())
        {
          data_in=Serial.read(); //Get next character 
          if(data_in=='X') InputX=Serial.parseInt();
          if(data_in=='Y') InputY=Serial.parseInt();
          if(data_in=='b') break; // End character
        }
      }
    }

    //Touch pad
    if(data_in=='A')
    { // Pad Start Text
      while(true)
      {
        if (Serial.available())
        {
          data_in=Serial.read(); //Get next character 
          if(data_in=='X') InputW=Serial.parseInt();
          if(data_in=='Y') InputZ=Serial.parseInt();
          if(data_in=='a') break; // End character
        }
      }
    }

    // Receive Data from Terminal Send Box
    if(data_in=='@')
    { // Start Text
     terminal_str="";
     while (data_in!=';')
     { // Loop until end text character reached
       if (Serial.available())
       {
         data_in=Serial.read();
         terminal_str+=data_in;
       }
     }
     printStr(">" + terminal_str);
     CMDRecived(terminal_str);
    }

  }

  ///////////// Send Data to Android device

  unsigned long t=millis();
  if ((t-last_time)>update_interval)
  {
    last_time=t;

    // Update Text Element 
    Serial.print("*0"+Outputs[0]+"*");

    // Update Text Element 
    Serial.print("*1"+Outputs[1]+"*");

    // Update Text Element 
    Serial.print("*3"+Outputs[3]+"*");

    // Update Text Element 
    Serial.print("*4"+Outputs[4]+"*");

    // Update Text Element 
    Serial.print("*2"+Outputs[2]+"*");


    // Green Progress Bar (Range is from 0 to 100)
    progress=map(battery_voltage, 0, 1023, 0, 100); // <--- Set Progress bar value here 
    Serial.print("*?"+String(progress)+"*");

    // Orange Progress Bar (Range is from 0 to 200)
    Serial.print("*Y"+String(map(InputW, -100, 100, 0, 200))+"*");

    // Orange Progress Bar (Range is from 0 to 200)
    Serial.print("*R"+String(map(InputX, -100, 100, 0, 200))+"*");

    // Bubble Gauge (Range is from 0 to 200)
    Serial.print("*T"+String(map(InputZ, -100, 100, 200, 0))+"*");

    // Bubble Gauge (Range is from 0 to 200)
    Serial.print("*P"+String(map(InputY, -100, 100, 200, 0))+"*");
  }
}

void CMDRecived(String CMD)
{

}

void calculate_angle() {

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_y * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_x * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (pi / 180). Transferring angels (math that I don't understand)
  angle_pitch -= angle_roll * sin(gyro_z * 0.000001066);
  angle_roll += angle_pitch * sin(gyro_z * 0.000001066);

  //Accelerometer angle calculations, Calculate the total accelerometer vector.
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

  if (abs(acc_y) < acc_total_vector) angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;       //Calculate the pitch angle.
  if (abs(acc_x) < acc_total_vector) angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc += 2.0;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc += 0.0;                                                    //Accelerometer calibration value for roll.

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

  pitch_level_adjust = angle_pitch * 10;
  roll_level_adjust = angle_roll * 10;
}

void read_mpu_6050_data() {
  //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       //Wait until all the bytes are received
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();

  gyro_y = gyro_y * -1;
  gyro_z = gyro_z * -1;
}

void setup_mpu_6050_registers() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();                                                    //End the transmission wit

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();

  //Let's perform a random register check to see if the values are written correct
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                                                          //Start reading @ register 0x1B
  Wire.endTransmission();                                                    //End the transmission
  Wire.requestFrom(0x68, 1);                                         //Request 1 bytes from the gyro
  while (Wire.available() < 1);                                              //Wait until the 6 bytes are received
  if (Wire.read() != 0x08) {                                                 //Check if the value is 0x08
    digitalWrite(12, HIGH);                                                  //Turn on the warning led
    while (1)delay(10);                                                      //Stay in this loop for ever
  }

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid() {
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void resetPID() {

  angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
  angle_roll = angle_roll_acc;
  set_gyro_angles = true;
  //reset all PID controllers
  pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_mem_yaw = 0;
  pid_last_yaw_d_error = 0;

}
