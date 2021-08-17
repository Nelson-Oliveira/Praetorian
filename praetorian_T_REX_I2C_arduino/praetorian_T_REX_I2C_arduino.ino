/*
  :PROJECT PRAETORIAN
  :Version 0.2.1
  :Author: Nelson Oliveira
  :Email: nfp.oliveira@gmail.com
  :Date: 
  :Last update: 
*/


////////////////////////////////////////////////////////////////// Turret

#include <Servo.h>

////////////////////////////////////////////////////////////////// Sharp IR sensor

#include <SharpIR.h>

////////////////////////////////////////////////////////////////// SR04 Sensor

#include <Ultrasonic.h>

////////////////////////////////////////////////////////////////// ROS

#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Range.h>

// 

#include <Wire.h>

ros::NodeHandle  nh;

#define LOOPTIME 10

////////////////////////////////////////////////////////////////// Sharp IR sensor

#define IRPinF A15         // Sharp IR
#define IRPinB A0         // Sharp IR
#define model 1080       // Sharp IR

////////////////////////////////////////////////////////////////// SR04 Sensor

#define pin_triggerF 3
#define pin_echoF 2
#define pin_triggerB 5
#define pin_echoB 4


int LOOPING = 40; //Loop for every 40 milliseconds.

void sensor_msg_init(sensor_msgs::Range &range_name, char *frame_id_name)
{
  range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_name.header.frame_id = frame_id_name;
  range_name.field_of_view = 0.26;
  range_name.min_range = 0.0;
  range_name.max_range = 2.0;
}
 
//Create four instances for range messages.
sensor_msgs::Range range_front;
sensor_msgs::Range range_back;
//sensor_msgs::Range range_frontbott;
//sensor_msgs::Range range_backbott;
 
//Create publisher objects for all sensors
ros::Publisher pub_range_front("/ultrasound_front", &range_front);
ros::Publisher pub_range_back("/ultrasound_back", &range_back);
//ros::Publisher pub_range_frontbott("/ultrasound_frontbott", &range_frontbott);
//ros::Publisher pub_range_backbott("/ultrasound_backbott", &range_backbott);


////////////////////////////////////////////////////////////////// T'Rex stuff
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

double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_act_right = 0;                    //Command speed for left wheel in m/s 

int ispeed_act_left = 0;
int ispeed_act_right = 0;

////////////////////////////////////////////////////////////////// SharpIR Range Finder

// Create variable to store the distance:
int Fdistance_cm;
int Bdistance_cm;
/* Model :
  GP2Y0A02YK0F --> 20150
  GP2Y0A21YK0F --> 1080
  GP2Y0A710K0F --> 100500
  GP2YA41SK0F --> 430
*/
// Create a new instance of the SharpIR class:
SharpIR frontIR = SharpIR(IRPinF, model);
SharpIR backIR = SharpIR(IRPinB, model);

///////////////////////////////////////////////////////

////////////////////*************** TEST Drive

int idemandx;
int idemandz;

float demandx=0;
float demandz=0;



void cmd_vel_cb( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  idemandx = abs(demandx*1000);
  Serial.print("IdemandX: ");
  Serial.print(idemandx);
  demandz = twist.angular.z;
  idemandz = abs(demandz*1000);
  Serial.print("IdemandZ: ");
  Serial.print(idemandz);
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type


////////////////////***************


////////////////////////////////////////////////////////////////// ROS shit



sensor_msgs::Range IRFrange_msg;
sensor_msgs::Range IRBrange_msg;
ros::Publisher pub_range_irfront( "range_data", &IRFrange_msg);
ros::Publisher pub_range_irback( "range_datab", &IRBrange_msg);

////////////////////////////////////////////////////////////////// SR04 Sensor
Ultrasonic ultrasonicF(pin_triggerF, pin_echoF);
Ultrasonic ultrasonicB(pin_triggerB, pin_echoB);


////////////////////////////////////////////////////////////////// Turret

Servo pan;  // create servo object to control a servo
Servo tilt;  // create servo object to control a servo

//int rcpan = 0;  // analog pin used to connect the potentiometer  potpin
//int rctilt = 0;  // analog pin used to connect the potentiometer  potpin2

int channel0;
int channel1;

int valpan;    // variable to read the value from the analog pin
int valtilt;

int valpanprev = 0;
int valtiltprev = 0;


void setup()
{
  Serial.begin(9600);
  Wire.begin();                                      // no address - join the bus as master
  nh.initNode();
  nh.advertise(pub_range_irfront);
  nh.advertise(pub_range_irback);
////////////////////////////////////////////////////////******************
  nh.subscribe(sub);
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic



////////////////////////////////////////////////////////******************


////////////////////////////////////////////////////////////////// Turret

  pan.attach(13);  // attaches the servo on pin 9 to the servo object
  tilt.attach(12);  // attaches the servo on pin 9 to the servo object

      //Define the input Pin for the Receiver
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  Serial.begin(9600);     //Serial comunication for later monitoring of channel signal value



////////////////////////////////////////////////////////////////// ROS SharpIR
  
  IRFrange_msg.radiation_type = sensor_msgs::Range::INFRARED;
  IRFrange_msg.header.frame_id =  frameid;
  IRFrange_msg.field_of_view = 0.01;
  IRFrange_msg.min_range = 0.1;  // For GP2D120XJ00F only. Adjust for other IR rangers
  IRFrange_msg.max_range = 0.8;   // For GP2D120XJ00F only. Adjust for other IR rangers

  IRBrange_msg.radiation_type = sensor_msgs::Range::INFRARED;
  IRBrange_msg.header.frame_id =  frameid;
  IRBrange_msg.field_of_view = 0.01;
  IRBrange_msg.min_range = 0.1;  // For GP2D120XJ00F only. Adjust for other IR rangers
  IRBrange_msg.max_range = 0.8;   // For GP2D120XJ00F only. Adjust for other IR rangers



///////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////// SR04 Sensor

 
  nh.initNode();
  nh.advertise(pub_range_front);
  nh.advertise(pub_range_back);
//  nh.advertise(pub_range_frontbott);
//  nh.advertise(pub_range_backbott);
 
  sensor_msg_init(range_front, "/ultrasound_front");
  sensor_msg_init(range_back, "/ultrasound_back");
//  sensor_msg_init(range_frontbott, "/ultrasound_frontbott");
//  sensor_msg_init(range_backbott, "/ultrasound_backbott");


}

void loop()
{

////////////////////////////////////////////////////////////********************************
  
  publishSpeed(LOOPTIME);

//////////////////////////////////////////////////////*******************************

  
                                                     // send data packet to T'REX controller 
  MasterSend(startbyte,2,lmspeed,lmbrake,rmspeed,rmbrake,devibrate,sensitivity,lowbat,idemandx,idemandz,i2caddr,i2cfreq);
  delay(50);
  MasterReceive();                                   // receive data packet from T'REX controller
  delay(50);

////////////////////////////////////////////////////////////////// Turret

  Turret();



////////////////////////////////////////////////////////////////// SharpIR
  Fdistance_cm = frontIR.distance();
  Bdistance_cm = backIR.distance();
  //  the measured distance to the serial monitor:
  Serial.print("Front IR distance: ");
  Serial.print(Fdistance_cm);
  Serial.println(" cm");
  Serial.print("Back IR distance: ");
  Serial.print(Bdistance_cm);
  Serial.println(" cm");
  delay(1000); //?? include this?



////////////////////////////////////////////////////////////////// ROS SharpIR rangefinder

  // publish the range value every 50 milliseconds
  //   since it takes that long for the sensor to stabilize
  if ( (millis()-range_timer) > 50){
    IRFrange_msg.range = Fdistance_cm;
    IRBrange_msg.range = Fdistance_cm;
    IRFrange_msg.header.stamp = nh.now();
    IRBrange_msg.header.stamp = nh.now();
    pub_range_irfront.publish(&IRFrange_msg);
    pub_range_irback.publish(&IRBrange_msg);
    range_timer =  millis();
  }



////////////////////////////////////////////////////////////////// SR04 Sensor

  //Le as informacoes do sensor, em cm e pol
  float cmMsecF, inMsecF;
  float cmMsecB, inMsecB;
  long microsecF = ultrasonicF.timing();
  long microsecB = ultrasonicB.timing();
  cmMsecF = ultrasonicF.convert(microsecF, Ultrasonic::CM);
  inMsecF = ultrasonicF.convert(microsecF, Ultrasonic::IN);
  cmMsecB = ultrasonicB.convert(microsecB, Ultrasonic::CM);
  inMsecB = ultrasonicB.convert(microsecB, Ultrasonic::IN);
  //Exibe informacoes no serial monitor
  Serial.print("Distancia em cm: ");
  Serial.print(cmMsecF);
  Serial.print(" - Distancia em polegadas: ");
  Serial.println(inMsecF);
  Serial.print("Distancia em cm: ");
  Serial.print(cmMsecB);
  Serial.print(" - Distancia em polegadas: ");
  Serial.println(inMsecB);
  delay(1000);

  range_front.range = cmMsecF;
  range_back.range = cmMsecB;
 
  range_front.header.stamp = nh.now();
  range_back.header.stamp = nh.now();
//  range_frontbott.header.stamp = nh.now();
//  range_backbott.header.stamp = nh.now();
 
  pub_range_front.publish(&range_front);
  pub_range_back.publish(&range_back);
//  pub_range_frontbott.publish(&range_frontbott);
//  pub_range_backbott.publish(&range_backbott);

  nh.spinOnce();

}

////////////////////////////////////////////////*************************************

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_act_left = ispeed_act_left;
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_act_right = ispeed_act_right;
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
//  nh.loginfo("Publishing odometry");
}

////////////////////////////////////////////////*************************************
