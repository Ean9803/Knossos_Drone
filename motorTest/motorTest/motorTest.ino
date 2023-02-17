
#include <Servo.h>

#define FREQ        250   // Sampling frequency

unsigned int  period; // Sampling period
unsigned long loop_timer;
unsigned long now, difference;

unsigned long pulse_length_esc1 = 1000,
pulse_length_esc2 = 1000,
pulse_length_esc3 = 1000,
pulse_length_esc4 = 1000;

int sensorValue = 0;

Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;
Servo ESC5;
Servo ESC6;

int pos = 0; //Sets position variable

void arm(){
setSpeed(ESC1, 0); //Sets speed variable delay(1000);
setSpeed(ESC2, 0); //Sets speed variable delay(1000);
setSpeed(ESC3, 0); //Sets speed variable delay(1000);
setSpeed(ESC4, 0); //Sets speed variable delay(1000);
setSpeed(ESC5, 0); //Sets speed variable delay(1000);
setSpeed(ESC6, 0); //Sets speed variable delay(1000);
}

void setSpeed(Servo ESC, int speed){
int angle = map(speed, 1000, 2000, 30, 180); //Sets servo positions to different speeds
ESC.write(angle);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ESC1.attach(4); //Adds ESC to certain pin.
  ESC2.attach(5); //Adds ESC to certain pin.
  ESC3.attach(6); //Adds ESC to certain pin.
  ESC4.attach(7); //Adds ESC to certain pin.
  ESC5.attach(8); //Adds ESC to certain pin.
  ESC6.attach(9); //Adds ESC to certain pin.
  arm(); 
}

void loop() {
  // put your main code here, to run repeatedly:
  setSpeed(ESC1, map(analogRead(A0), 0, 1023, 1000, 2000));
  setSpeed(ESC2, map(analogRead(A1), 0, 1023, 1000, 2000));
  setSpeed(ESC3, map(analogRead(A2), 0, 1023, 1000, 2000));
  setSpeed(ESC4, map(analogRead(A3), 0, 1023, 1000, 2000));
  
}

void SetUpTimedProcess()
{
  DDRD |= B11110000;
  // Configure interrupts for receiver
  PCICR |= (1 << PCIE0);  // Set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0); // Set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1); // Set PCINT1 (digital input 9) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2); // Set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3); // Set PCINT3 (digital input 11)to trigger an interrupt on state change

  period = 20000; // Sampling period in µs
  pinMode(A0, INPUT);
  loop_timer = micros();
}

void RunTimedProcess()
{
  applyMotorSpeed();
  pulse_length_esc1 = 
  pulse_length_esc2 = 
  pulse_length_esc3 =
  pulse_length_esc4 =
  sensorValue = map(analogRead(A0), 0, 1023, 1000, 2000);
}


void applyMotorSpeed()
{
    // Refresh rate is 250Hz: send ESC pulses every 4000µs
    if ((now = micros()) - loop_timer >= period)
    {
    // Update loop timer
    loop_timer = now;
    PORTD |= B11110000;
    }
    else
    {
    // Wait until all pins #4 #5 #6 #7 are LOW
        now = micros();
        difference = now - loop_timer;

        if (difference >= pulse_length_esc1) PORTD &= B11101111; // Set pin #4 LOW
        if (difference >= pulse_length_esc2) PORTD &= B11011111; // Set pin #5 LOW
        if (difference >= pulse_length_esc3) PORTD &= B10111111; // Set pin #6 LOW
        if (difference >= pulse_length_esc4) PORTD &= B01111111; // Set pin #7 LOW
    }
}
