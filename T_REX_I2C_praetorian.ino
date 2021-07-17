#include <SharpIR.h>
#include <ros.h>
#include <ros/time.h>
////////////////////////////////////////////////////****************
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>
#include <geometry_msgs/Vector3Stamped.h>
////////////////////////////////////////////////////****************

#include <sensor_msgs/Range.h>
#include <Wire.h>

ros::NodeHandle  nh;

#define LOOPTIME 10  ////////////****************

#define IRPin A0         // Sharp IR
#define model 1080       // Sharp IR


/////////////////////////////////////////////////////// T'Rex stuff
#define startbyte 0x0F
#define I2Caddress 0x07


int sv[6]={1500,1500,1500,1500,0,0};                 // servo positions: 0 = Not Used
int sd[6]={5,10,-5,-15,20,-20};                      // servo sweep speed/direction
int lmspeed,rmspeed;                                 // left and right motor speed from -255 to +255 (negative value = reverse)
int ldir=5;                                          // how much to change left  motor speed each loop (use for motor testing)
int rdir=5;                                          // how much to change right motor speed each loop (use for motor testing)
byte lmbrake,rmbrake;                                // left and right motor brake (non zero value = brake)
byte devibrate=50;                                   // time delay after impact to prevent false re-triggering due to chassis vibration
int sensitivity=50;                                  // threshold of acceleration / deceleration required to register as an impact
int lowbat=650;                                      // adjust to suit your battery: 550 = 5.50V
byte i2caddr=7;                                      // default I2C address of T'REX is 7. If this is changed, the T'REX will automatically store new address in EEPROM
byte i2cfreq=0;                                      // I2C clock frequency. Default is 0=100kHz. Set to 1 for 400kHz


unsigned long range_timer;
char frameid[] = "/ir_ranger";


/////////////////////////////////////////////////////// SharpIR Range Finder

// Create variable to store the distance:
int distance_cm;
/* Model :
  GP2Y0A02YK0F --> 20150
  GP2Y0A21YK0F --> 1080
  GP2Y0A710K0F --> 100500
  GP2YA41SK0F --> 430
*/
// Create a new instance of the SharpIR class:
SharpIR mySensor = SharpIR(IRPin, model);

///////////////////////////////////////////////////////

////////////////////*************** TEST Drive

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

double left_kp = 3.8 , left_ki = 0 , left_kd = 0.0;             // modify for optimal performance
double right_kp = 4 , right_ki = 0 , right_kd = 0.0;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);  

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  

float demandx=0;
float demandz=0;

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

void cmd_vel_cb( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_act_right = 0;                    //Command speed for left wheel in m/s 
////////////////////***************


/////////////////////////////////////////////////////// ROS shit



sensor_msgs::Range range_msg;
ros::Publisher pub_range( "range_data", &range_msg);


void setup()
{
  Serial.begin(9600);
  Wire.begin();                                      // no address - join the bus as master
  nh.initNode();
  nh.advertise(pub_range);
////////////////////////////////////////////////////////******************
  nh.subscribe(sub);
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic

  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(-100, 100);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(1);
  leftPID.SetOutputLimits(-100, 100);

////////////////////////////////////////////////////////******************

////////////////////////////////////////////////////// ROS SharpIR
  
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.01;
  range_msg.min_range = 0.1;  // For GP2D120XJ00F only. Adjust for other IR rangers
  range_msg.max_range = 0.8;   // For GP2D120XJ00F only. Adjust for other IR rangers



///////////////////////////////////////////////////////////////////////////////////

}

void loop()
{

////////////////////////////////////////////////////////////********************************
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME){
    prevMillis = currentMillis;

    demand_speed_left = demandx - (demandz*0.1075);
    demand_speed_right = demandx + (demandz*0.1075);
  
    /*PID controller for speed control
      Base speed being 1 ms and the demand_speed variables controlling it at fractions of the base.
      The PID controller keeps trying to match the difference 
      in encoder counts to match with the required amount, hence controlling the speed. */
    encoder0Diff = encoder0Pos - encoder0Prev; // Get difference between ticks to compute speed
    encoder1Diff = encoder1Pos - encoder1Prev;
    
    speed_act_left = encoder0Diff/39.65;                    
    speed_act_right = encoder1Diff/39.65; 
  
    encoder0Error = (demand_speed_left*39.65)-encoder0Diff; // 3965 ticks in 1m = 39.65 ticks in 10ms, due to the 10 millis loop
    encoder1Error = (demand_speed_right*39.65)-encoder1Diff;
  
    encoder0Prev = encoder0Pos; // Saving values
    encoder1Prev = encoder1Pos;
  
    left_setpoint = demand_speed_left*39.65;  //Setting required speed as a mul/frac of 1 m/s
    lmspeed = left_setpoint;
    right_setpoint = demand_speed_right*39.65;
    rmspeed = right_setpoint;
  
    left_input = encoder0Diff;  //Input to PID controller is the current difference
    right_input = encoder1Diff;
    
    leftPID.Compute();
//    left.rotate(left_output);
    rightPID.Compute();
//    right.rotate(right_output);
//    Serial.print(encoder0Pos);
//    Serial.print(",");
//    Serial.println(encoder1Pos);
  }
  publishSpeed(LOOPTIME);

//////////////////////////////////////////////////////*******************************

  
                                                     // send data packet to T'REX controller 
  MasterSend(startbyte,2,lmspeed,lmbrake,rmspeed,rmbrake,sv[0],sv[1],sv[2],sv[3],sv[4],sv[5],devibrate,sensitivity,lowbat,i2caddr,i2cfreq);
  delay(50);
  MasterReceive();                                   // receive data packet from T'REX controller
  delay(50);

//////////////////////////////////////////////////////////////////// SharpIR
  distance_cm = mySensor.distance();
  // Print the measured distance to the serial monitor:
  Serial.print("Mean distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
  delay(1000); //?? include this?



// ROS IR rangefinder

  // publish the range value every 50 milliseconds
  //   since it takes that long for the sensor to stabilize
  if ( (millis()-range_timer) > 50){
    range_msg.range = distance_cm;
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_timer =  millis();
  }

 
  nh.spinOnce();


  
/*
  //=================================================== Code to test motors and sweep servos =============================================  
  lmspeed+=ldir;
  if(lmspeed>240 or lmspeed<-240) ldir=-ldir;        // increase / decrease left motor speed and direction (negative values = reverse direction)
  
  rmspeed+=rdir;
  if(rmspeed>240 or rmspeed<-240) rdir=-rdir;        // increase / decrease left motor speed and direction (negative values = reverse direction)
  
  lmbrake=(abs(lmspeed)>235);                        // test left  motor brake 
  rmbrake=(abs(rmspeed)>235);                        // test right motor brake 
  
  for(byte i=0;i<6;i++)                              // sweep servos
  {
    if(sv[i]!=0)                                     // a value of 0 indicates no servo attached
    {
      sv[i]+=sd[i];                                  // update servo position to create sweeping motion
      if(sv[i]>2000 || sv[i]<1000) sd[i]=-sd[i];     // reverse direction of servo if limit reached
    }
  }
*/  
}

////////////////////////////////////////////////*************************************

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
//  nh.loginfo("Publishing odometry");
}

////////////////////////////////////////////////*************************************
