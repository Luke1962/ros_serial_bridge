/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
interrupt pin MEGA: 2,3,  18,19,  20,21
 */

///////////////////////////////////////////////////////////////////////
///	ROS
///////////////////////////////////////////////////////////////////////
#define USE_ARDUINOHARDWARE
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>	//rwheel (std_msgs/Int16)
//#include <std_msgs/Int32.h>
//#define ROSSERIAL_BAUD_RATE 115200
#define ROSSERIAL_BAUD_RATE 57600
//#define ROSSERIAL_BAUD_RATE 19200

#define PUBLISH_ENCTICKS
///////////////////////////////////////////////////////////////////////
///	DEBUG
///////////////////////////////////////////////////////////////////////
//#define DEBUG_OFF
#define DEBUG_ON

#ifdef DEBUG_ON
	#define DEBUG_SERIAL Serial1
	#define DEBUG_SERIAL_BAUD_RATE 9600

	#define dbgF(t)		 DEBUG_SERIAL.println(F(t));	 DEBUG_SERIAL.flush() ;
	#define dbg(v)		 DEBUG_SERIAL.print(v);	
	#define dbg2(t,v)	 DEBUG_SERIAL.print(F(t));DEBUG_SERIAL.println(v);  
	// #define dbg2(t,cha)    SERIAL_PC.print(t);SERIAL_PC.println(cha);//  SERIAL_SPEAK.print(t);SERIAL_SPEAK.println(cha);  

#else
	#define dbg(cha)
	#define dbg2(t,cha)	

#endif // DEBUG_ON

///////////////////////////////////////////////////////////////////////
///	Servo
///////////////////////////////////////////////////////////////////////
#if 1
	#define SERVO
	#define SERVO_MIN 5
	#define SERVO_MAX 175
	#define SERVO_DEFAULT 140 // 180-alfa  
	#include <Servo.h>
	#include <math.h>
	Servo myservo;  // create servo object to control a servo
	int pos = 0;    // variable to store the servo position
	bool blServoPositionChanged = true;
	int servoPos_actual;
	int servoPos_demand;

	void setup_servo() {
		myservo.attach(9);  // attaches the servo on pin 9 to the servo object
	}
	// va alla posizione richiesta in modo smooth (per il flat cable della raspicam)
	void myservoGoTo(int  servoPos_demand){
		servoPos_actual = myservo.read();
		dbg2("servoPos_actual: ",servoPos_actual);
		dbg2("servoPos_demand: ",servoPos_demand);

		// limiter
		if (servoPos_demand < SERVO_MIN){ servoPos_demand = SERVO_MIN;		}
		if (servoPos_demand > SERVO_MAX){ servoPos_demand = SERVO_MAX;		}

		// signal newposition
		if (servoPos_demand != servoPos_actual) {
			int delta = servoPos_demand -servoPos_actual;
			dbg2("  Steps of", delta/10);
			while (servoPos_actual != servoPos_demand){

				if (abs(servoPos_demand - servoPos_actual) < 20) {
					myservo.write(servoPos_demand);  // vai direttamente
					delay(30);
					servoPos_actual = myservo.read();
				}else{
					// divido in più passi
					int x = servoPos_actual +(delta/10);
					dbg2("  x:",x);
					myservo.write(x);              // tell servo to go to position in variable 'pos'
					delay(50); // waits 15ms for the servo to reach the position
					servoPos_actual = myservo.read();
				}	

			}
		
			myservo.write(servoPos_demand);  
			delay(20);
			servoPos_actual = myservo.read();
			
			blServoPositionChanged = true;	

		}
	}

	void servoSweep() {
		myservoGoTo(SERVO_MIN);
		delay(3000);
		myservoGoTo(SERVO_MAX);
		delay(3000);
		myservoGoTo(SERVO_DEFAULT);
	}

#endif
///////////////////////////////////////////////////////////////////////
///	Encoder
///////////////////////////////////////////////////////////////////////

#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder encL(2, 4);
Encoder encR(3, 5);
//   avoid using pins with LEDs attached
///////////////////////////////////////////////////////////////////////

#define DBG Serial1
///////////////////////////////////////////////////////////////////////
///	ROS
///////////////////////////////////////////////////////////////////////
#define USE_ARDUINOHARDWARE

#define ENC_TOPIC "encoders"
#define ROS_INFO(s) nh.loginfo(s);

// questo è come in espRosScan che funziona
ros::NodeHandle nh; // con aggiunta in Ros.h di: 	typedef NodeHandle_<Esp8266Hardware, 25, 25, 512, 1024> NodeHandle;
geometry_msgs::Vector3Stamped msg_encoders;
ros::Publisher pub_encoders(ENC_TOPIC, &msg_encoders);
int delayBetweenLoops_ms =30; 
#ifdef PUBLISH_ENCTICKS
	std_msgs::Int16 msg_lwheel;
	std_msgs::Int16 msg_rwheel;
	ros::Publisher pub_lwheel("/lwheel", &msg_lwheel);
	ros::Publisher pub_rwheel("/rwheel", &msg_rwheel);
	
#endif //PUBLISH_ENCTICKS


void cbkEncReset(const std_msgs::Empty &msg){
	encL.write(0);
	encR.write(0);
}

ros::Subscriber<std_msgs::Empty> sub_encReset("encoders_reset", &cbkEncReset);


///////////////////////////////////////////////////////////////////////
///	FARETTO
///////////////////////////////////////////////////////////////////////
#define PIN_FARETTO 12
#define FARETTO_ON digitalWrite(PIN_FARETTO, 1);
#define FARETTO_OFF digitalWrite(PIN_FARETTO, 0);

void cbkFarettoOnOff(const std_msgs::Bool &msg){
	digitalWrite(PIN_FARETTO,msg.data);
}
ros::Subscriber<std_msgs::Bool> sub_faretto("faretto", &cbkFarettoOnOff);

//-----------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////
///	LASER
///////////////////////////////////////////////////////////////////////
#define PIN_LASER 11
#define LASER_ON digitalWrite(PIN_LASER, 1);
#define LASER_OFF digitalWrite(PIN_LASER, 0);

void cbk_laser(const std_msgs::Bool &msg){
	digitalWrite(PIN_LASER,msg.data);
}
ros::Subscriber<std_msgs::Bool> sub_laser("laser", &cbk_laser);

//-----------------------------------------------------------------------



#ifdef SERVO
	std_msgs::Int16 msg_raspicam_servo_position;
	ros::Publisher pub_raspicamservo("/raspicam_servo_position", &msg_raspicam_servo_position);


	void cbkServo(const std_msgs::Int16 &msg){		 
		myservoGoTo(180 - msg.data); // 180 =raspicam orizzontale in avanti 
	}	


	ros::Subscriber<std_msgs::Int16> sub_servo("/raspicam_tilt_demand", &cbkServo);

#endif //SERVO


#define LED_ON digitalWrite(LED_BUILTIN, 1);
#define LED_OFF digitalWrite(LED_BUILTIN, 0);

inline void setup_ros(){
	DBG.println("Waiting Rosserial...");
	nh.initNode();
/* 	
	while(!nh.connected()){
		nh.spinOnce();
		delay(100);
	}
 */	

	nh.advertise(pub_encoders);
	nh.advertise(pub_lwheel);
	nh.advertise(pub_rwheel);
	nh.advertise(pub_raspicamservo);
	nh.subscribe(sub_encReset);	
	nh.subscribe(sub_servo);  
	nh.subscribe(sub_faretto);
	nh.subscribe(sub_laser);
	
	while(!nh.connected()) nh.spinOnce();
  	nh.loginfo("Startup complete");
	DBG.println("CONNESSO");
	int DELAY_DEFAULT = 3;
	/* 	
	if (! nh.getParam("~delay", &delayBetweenLoops_ms, &DELAY_DEFAULT,1)){ 
    	//default values
    	delayBetweenLoops_ms= 30;
		dbg2("No param rate, default=",DELAY_DEFAULT);
	}else
	{
		dbg2("Parameter: delayBetweenLoops_ms=",delayBetweenLoops_ms);
	}
	 */

	// Get delay parameter------------------
	bool param_success = false;
	dbg("loading param. ~delay...");
	while (!param_success)
	{
		param_success = nh.getParam("~delay", (int*)&delayBetweenLoops_ms, 1);
		nh.spinOnce();
		delay(10);
	}
	dbg2("...ok delay=",delayBetweenLoops_ms);


}

void setup()
{
	DBG.begin(DEBUG_SERIAL_BAUD_RATE);
	Serial.begin(ROSSERIAL_BAUD_RATE);
	DBG.println("\n-----ROS Encoder node start-------");
	dbg2("Baud rate:", ROSSERIAL_BAUD_RATE);

	// pilotaggio faretto
	pinMode(PIN_FARETTO ,OUTPUT);
	FARETTO_OFF;
	// Pilotaggio LASER
	pinMode(PIN_LASER ,OUTPUT);
	LASER_OFF;

	LED_ON;
	setup_ros();
	LED_OFF
	dbg("ROS initNode done");

	setup_servo();
	servoSweep();


}

#define print_dbg_interval_ms 1000
unsigned long last_print_dbg = millis();
long oldPosL = -999;
long oldPosR = -999;
long newPosL ;
long newPosR ;
void loop()
{
	newPosL = encL.read();
	newPosR = encR.read();
	msg_encoders.header.stamp = nh.now();
	msg_encoders.vector.x = newPosL; //thisSystem.encL.count;
	msg_encoders.vector.y = newPosR; // thisSystem.encR.count;
	pub_encoders.publish(&msg_encoders);

	#ifdef PUBLISH_ENCTICKS
		msg_lwheel.data = newPosL;
		msg_rwheel.data = newPosR;
		pub_lwheel.publish(&msg_lwheel);
		pub_rwheel.publish(&msg_rwheel);
	#endif //PUBLISH_ENCTICKS


	if ( ((newPosL != oldPosL)||(newPosR != oldPosR) ) &&
		(millis() > last_print_dbg + print_dbg_interval_ms)
		) {
    	oldPosL = newPosL;
    	oldPosR = newPosR;
   		DBG.print("EncL  ");DBG.print(newPosL);
		DBG.print(",R ");DBG.print(newPosR);
		DBG.print(" , Diff ");DBG.println(abs(newPosL)-abs(newPosR));
		last_print_dbg =millis();
  	} 

/* 	#ifdef SERVO
		 if (blServoPositionChanged) {
			msg_raspicam_servo_position.data =servoPos_actual;
			pub_raspicamservo.publish(&msg_raspicam_servo_position);
			blServoPositionChanged = false;
		 }
		 
	#endif //SERVO
 */
	nh.spinOnce();
	delay(delayBetweenLoops_ms);

}
