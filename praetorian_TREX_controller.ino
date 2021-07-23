
//=====================================================================================================================================//
//                     __________________   ____     ______________        __________________   _____         ______                   //
//                    /                 /  /    |   /              \      /                 /   \    \      _/    _/                   //
//                   /____      _______/  /_   /   /    _______     \    /    _____________/     \    \   _/    _/                     //
//                        /    /            | /   /    /      /     /   /    /_____               \    \_/    _/                       //
//                       /    /             |/   /    /______/     /   /          /              _/         _/                         //
//                      /    /                  /    ___     _____/   /    ______/             _/    _     /                           //
//                     /    /                  /    /   \    \       /    /____________      _/    _/ \    \                           //
//                    /    /                  /    /     \    \     /                 /    _/    _/    \    \                          //
//                   /____/                  /____/       \____\   /_________________/    /_____/       \____\                         //
//                                                                                                                                     //
//                 T'REX robot controller designed and programmed by Russell Cameron for DAGU Hi-Tech Electronics                      //
//=====================================================================================================================================//


#include <Wire.h>                                      // interrupt based I2C library
#include <Servo.h>                                     // library to drive up to 12 servos using timer1
#include <EEPROM.h>                                    // library to access EEPROM memory
#include "IOpins.h"                                    // defines which I/O pin is used for what function

#include <PID_v1.h>

#define LOOPTIME 10  ////////////****************
// define constants here
#define startbyte 0x0F                                 // for serial communications each datapacket must start with this byte

// define global variables here
byte mode=0;                                           // mode=0: I2C / mode=1: Radio Control / mode=2: Bluetooth / mode=3: Shutdown
int  lowbat=650;                                       // default low battery voltage is 5.5V
byte errorflag;                                        // non zero if bad data packet received
byte pwmfreq;                                          // value from 1-7
byte i2cfreq;                                          // I2C clock frequency can be 100kHz(default) or 400kHz
byte I2Caddress;                                       // I2C slave address
int lmspeed,rmspeed;                                   // left and right motor speeds -255 to +255
byte lmbrake,rmbrake;                                  // left and right brakes - non zero values enable brake
int lmcur,rmcur;                                       // left and right motor current
int lmenc,rmenc;                                       // left and right encoder values
int volts;                                             // battery voltage*10 (accurate to 1 decimal place)
int xaxis,yaxis,zaxis;                                 // X, Y, Z accelerometer readings
int deltx,delty,deltz;                                 // X, Y, Z impact readings 
int magnitude;                                         // impact magnitude
byte devibrate=50;                                     // number of 2mS intervals to wait after an impact has occured before a new impact can be recognized
int sensitivity=50;                                    // minimum magnitude required to register as an impact

byte RCdeadband=35;                                    // RCsignal can vary this much from 1500uS without controller responding
unsigned long time;                                    // timer used to monitor accelerometer and encoders

//byte servopin[6]={7,8,12,13,5,6};                      // array stores IO pin for each servo
//int servopos[6];                                       // array stores position data for up to 6 servos
//Servo servo[6];                                        // create 6 servo objects as an array

float demandx=0;
float demandz=0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

volatile long encoder0Pos = 0;    // encoder left  ---lmec
volatile long encoder1Pos = 0;    // encoder right ---rmenc


double left_kp = 0 , left_ki = 0 , left_kd = 0.0;             // modify for optimal performance
double right_kp = 0 , right_ki = 0 , right_kd = 0.0;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);  

// rmspeed = (int) right_output
// lmspeed = (int) left_output



double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  

double demand_speed_left;
double demand_speed_right;

unsigned long currentMillis;
unsigned long prevMillis;

float encoder0Diff;
float encoder1Diff;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;

double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_act_right = 0;                    //Command speed for left wheel in m/s 

int ispeed_act_left;
int ispeed_act_right;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  //========================================== Choose your desired motor PWM frequency ================================================//
  //                       Note that higher frequencies increase inductive reactance and reduce maximum torque                         //
  //                               Many smaller motors will not work efficiently at higher frequencies                                 //
  //                      The default is 122Hz. This provides relatively low noise and relatively smooth torque                        //
  //                                    This setting can be changed using I2C or Bluetooth                                             //
  //                                                                                                                                   //
  //     Thanks to macegr - http://forum.arduino.cc/index.php?PHPSESSID=n1691l4esq4up52krpcb77bgm1&topic=16612.msg121031#msg121031     //
  //===================================================================================================================================//

  //TCCR2B = TCCR2B & B11111000 | B00000001; pwmfreq=1;    // set timer 2 divisor to    1 for PWM frequency of  31250.000000000 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000010; pwmfreq=2;    // set timer 2 divisor to    8 for PWM frequency of   3906.250000000 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011; pwmfreq=3;    // set timer 2 divisor to   32 for PWM frequency of    976.562500000 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000100; pwmfreq=4;    // set timer 2 divisor to   64 for PWM frequency of    488.281250000 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000101; pwmfreq=5;    // set timer 2 divisor to  128 for PWM frequency of    244.140625000 Hz
    TCCR2B = TCCR2B & B11111000 | B00000110; pwmfreq=6;    // set timer 2 divisor to  256 for PWM frequency of    122.070312500 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111; pwmfreq=7;    // set timer 2 divisor to 1024 for PWM frequency of     30.517578125 Hz


  //all IO pins are input by default on powerup --------- configure motor control pins for output -------- pwm autoconfigures -----------

  pinMode(lmpwmpin,OUTPUT);                            // configure left  motor PWM       pin for output
  pinMode(lmdirpin,OUTPUT);                            // configure left  motor direction pin for output
  pinMode(lmbrkpin,OUTPUT);                            // configure left  motor brake     pin for output
  
  pinMode(rmpwmpin,OUTPUT);                            // configure right motor PWM       pin for output
  pinMode(rmdirpin,OUTPUT);                            // configure right motor direction pin for output
  pinMode(rmbrkpin,OUTPUT);                            // configure right motor brake     pin for output


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(-255, 255);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(1);
  leftPID.SetOutputLimits(-255, 255);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  
  //----------------------------------------------------- Test for RC inputs ------------------------------------------------------------

  digitalWrite(RCspeedpin,1);                          // enable weak pullup resistor on input to prevent false triggering                   
  digitalWrite(RCsteerpin,1);                          // enable weak pullup resistor on input to prevent false triggering
  delay(100);
  int t1=int(pulseIn(RCspeedpin,HIGH,30000));          // read throttle/left stick
  int t2=int(pulseIn(RCsteerpin,HIGH,30000));          // read steering/right stick
  if(t1>1000 && t1<2000 && t2>1000 && t2<2000)         // RC signals detected - go to RC mode
  {
    mode=1;                                            // set mode to RC
    MotorBeep(3);                                      // generate 3 beeps from the motors to indicate RC mode enabled
  }
  
  //----------------------------------------------------- Test for Bluetooth module ------------------------------------------------------
  if(mode==0)                                          // no RC signals detected
  {
    BluetoothConfig();                                 // attempts to configure bluetooth module - changes to mode 2 if successful
    if(mode==2) MotorBeep(2);                          // generate 2 beeps from the motors to indicate bluetooth mode enabled
  }
  
  //----------------------------------------------------- Configure for I²C control ------------------------------------------------------
  if(mode==0)                                          // no RC signal or bluetooth module detected
  {
    MotorBeep(1);                                      // generate 1 beep from the motors to indicate I²C mode enabled
    byte i=EEPROM.read(0);                             // check EEPROM to see if I²C address has been previously stored
    if(i==0x55)                                        // B01010101 is written to the first byte of EEPROM memory to indicate that an I2C address has been previously stored
    {
      I2Caddress=EEPROM.read(1);                       // read I²C address from EEPROM
    }
    else                                               // EEPROM has not previously been used by this program
    {
      EEPROM.write(0,0x55);                            // set first byte to 0x55 to indicate EEPROM is now being used by this program
      EEPROM.write(1,0x07);                            // store default I²C address
      I2Caddress=0x07;                                 // set I²C address to default
    }
    
    Wire.begin(I2Caddress);                            // join I²C bus as a slave at I2Caddress
    Wire.onReceive(I2Ccommand);                        // specify ISR for data received
    Wire.onRequest(I2Cstatus);                         // specify ISR for data to be sent
  }

}


void loop()
{
  //----------------------------------------------------- Diagnostic mode --------------------------------------------------------------
  /*
  DiagnosticMode();
  return;
  */
  //----------------------------------------------------- Shutdown mode ----------------------------------------------------------------
  if (mode==3)                                         // if battery voltage too low
  {
    Shutdown();                                        // Shutdown motors and servos
    return;
  }
  
  //----------------------------------------------------- RC Mode -----------------------------------------------------------------------
  if(mode==1)                                           
  {
    RCmode();                                          // monitor signal from RC receiver and control motors
    return;                                            // I²C, Bluetooth and accelerometer are ignored
  }
  
  //----------------------------------------------------- Bluetooth mode ----------------------------------------------------------------
  if(mode==2)
  {
    Bluetooth();                                       // control using Android phone and sample app
    return;
  }
  
  //----------------------------------------------------- I²C mode ----------------------------------------------------------------------
  

  //===================================================== Programmer's Notes ============================================================
  //                                    Detecting impacts requires reasonably accurate timing.                                         //
  //                  As all timers are in use this code uses the micros() function to simulate a timer interrupt.                     // 
  //                                                                                                                                   //
  //                      Reading an analog input takes 260uS so reading 3 analog inputs can take about 800uS.                         //
  //                 This code alternates between reading accelerometer data and voltage / current readings every 1mS.                 //
  //                  If you edit this code then be aware that impact detection may be affected if care is not taken.                  //
  //=====================================================================================================================================


  static byte alternate;                               // variable used to alternate between reading accelerometer and power analog inputs
  
  
  
  //----------------------------------------------------- Perform these functions every 1mS ---------------------------------------------- 
  if(micros()-time>999)                       
  {
    time=micros();                                     // reset timer
    alternate=alternate^1;                             // toggle alternate between 0 and 1
    Encoders();                                        // check encoder status every 1mS

    //--------------------------------------------------- These functions must alternate as they both take in excess of 780uS ------------    
    if(alternate)
    {
      Accelerometer();                                 // monitor accelerometer every second millisecond                            
    }
    else 
    {
      lmcur=(analogRead(lmcurpin)-511)*48.83;          // read  left motor current sensor and convert reading to mA
      rmcur=(analogRead(rmcurpin)-511)*48.83;          // read right motor current sensor and convert reading to mA
      volts=analogRead(voltspin)*10/3.357;             // read battery level and convert to volts with 2 decimal places (eg. 1007 = 10.07 V)
      if(volts<lowbat) mode=3;                         // change to shutdown mode if battery voltage too low
    }
  }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME){
    prevMillis = currentMillis;

    demand_speed_left = demandx - (demandz*0.1075);
    demand_speed_right = demandx + (demandz*0.1075);
  
    /*PID controller for speed control
      Base speed being 1 ms and the demand_speed variables controlling it at fractions of the base.
      The PID controller keeps trying to match the difference 
      in encoder counts to match with the required amount, hence controlling the speed. */
    encoder0Diff = lmenc - encoder0Prev; // Get difference between ticks to compute speed
    encoder1Diff = rmenc - encoder1Prev;
    
    speed_act_left = encoder0Diff/39.65;  
    ispeed_act_left = abs(speed_act_left*1000);                    
    speed_act_right = encoder1Diff/39.65; 
    ispeed_act_right = abs(speed_act_right*1000); 
  
    encoder0Error = (demand_speed_left*39.65)-encoder0Diff; // 3965 ticks in 1m = 39.65 ticks in 10ms, due to the 10 millis loop
    encoder1Error = (demand_speed_right*39.65)-encoder1Diff;
  
    encoder0Prev = lmenc; // Saving values
    encoder1Prev = rmenc;
  
    left_setpoint = demand_speed_left*39.65;  //Setting required speed as a mul/frac of 1 m/s
    right_setpoint = demand_speed_right*39.65;
  
    left_input = encoder0Diff;  //Input to PID controller is the current difference
    right_input = encoder1Diff;
    
    leftPID.Compute();
    lmspeed = abs(left_output);
    rightPID.Compute();
    rmspeed = abs(right_output);
//    Serial.print(lmenc);
//    Serial.print(",");
//    Serial.println(rmenc);
  }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// run every 10 ms

//targetspeed = 1 //m/s

// 1 m = 10000 encoder counts

//está a 1000 estava a 800 -> 10ms fizeste 200 ticks 200/10000 dagu andou 0.02m em 10ms -> velocidade atual é 2m/s
  
}
