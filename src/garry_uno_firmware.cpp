#include "garry_uno_firmware.h"

// comment out to disable logging
//#define ENABLE_LOG

#define MAX_LOG_LEN 30

/* global variables */
Servo servoA;
Servo servoB;

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Bool> sub_led_right_eye("led_right_eye", &led_right_eye_cb);
ros::Subscriber<std_msgs::Bool> sub_led_left_eye("led_left_eye", &led_left_eye_cb);

ros::Subscriber<geometry_msgs::Twist> sub_motor_left("motor_left", &motor_left_cb);
ros::Subscriber<geometry_msgs::Twist> sub_motor_right("motor_right", &motor_right_cb);
ros::Subscriber<geometry_msgs::Twist> sub_motor_both("motor_both", &motor_both_cb);

ros::Subscriber<geometry_msgs::Twist> sub_servo_upper("servo_upper", &servo_upper_cb);
ros::Subscriber<geometry_msgs::Twist> sub_servo_lower("servo_lower", &servo_lower_cb);

void setup() {

    /* initialise motor shield */

    //Setup Channel A
    pinMode(12, OUTPUT); //Initiates Motor Channel A pin
    pinMode(9, OUTPUT); //Initiates Brake Channel A pin

    //Setup Channel B
    pinMode(13, OUTPUT); //Initiates Motor Channel A pin
    pinMode(8, OUTPUT);  //Initiates Brake Channel A pin


    /* initialise servos */
    servoA.attach(5);
    servoB.attach(6);

    /* initialise LEDs */
    pinMode(4, OUTPUT);
    pinMode(7, OUTPUT);
    
    /* initialising ROS */
    nh.initNode();
    nh.subscribe(sub_led_left_eye);
    nh.subscribe(sub_led_right_eye);
    nh.subscribe(sub_motor_left);
    nh.subscribe(sub_motor_right);
    nh.subscribe(sub_servo_upper);
    nh.subscribe(sub_servo_lower);
    nh.subscribe(sub_motor_both);
}

/* main loop */
void loop() {
	
	nh.spinOnce();
	delay(1000);
}

/* callback functions */

void led_left_eye_cb(const std_msgs::Bool& msg) {
	
	if (msg.data == 1)
		digitalWrite(4, HIGH);
	else
		digitalWrite(4, LOW);
	
	#ifdef ENABLE_LOG
	char logoutput[MAX_LOG_LEN];
	snprintf(logoutput,MAX_LOG_LEN,"Setting left eye %d",msg.data);
	nh.loginfo(logoutput);
	#endif
}

void led_right_eye_cb(const std_msgs::Bool& msg) {
	
	if (msg.data == 1)
		digitalWrite(7, HIGH);
	else
		digitalWrite(7, LOW);
	
	#ifdef ENABLE_LOG
	char logoutput[MAX_LOG_LEN];
	snprintf(logoutput,MAX_LOG_LEN,"Setting right eye %d",msg.data);
	nh.loginfo(logoutput);
	#endif
}

void motor_left_cb(const geometry_msgs::Twist& msg) {

	uint8_t speed = (uint8_t) (255 * msg.linear.x);
	
	#ifdef ENABLE_LOG
	char logoutput[MAX_LOG_LEN];
	snprintf(logoutput,MAX_LOG_LEN,"Left motor moving, speed: %d",speed);
	nh.loginfo(logoutput);
	#endif

	if (msg.linear.x > 0)
		digitalWrite(12, LOW); // A motor forward

	else
		digitalWrite(12, HIGH); // A motor backwards	
	
	
	digitalWrite(9, LOW); // disable A motor brake
	analogWrite(3, speed);

	delay(1000);

	digitalWrite(9, HIGH); //enable motor A brake
}

void motor_right_cb(const geometry_msgs::Twist& msg) {

	uint8_t speed = (uint8_t) (255 * abs(msg.linear.x));
	
	#ifdef ENABLE_LOG
	char logoutput[MAX_LOG_LEN];
	snprintf(logoutput,MAX_LOG_LEN,"Right motor moving, speed: %d",speed);
	nh.loginfo(logoutput);
	#endif

	if (msg.linear.x > 0)
		digitalWrite(13, HIGH); // B motor forward

	else
		digitalWrite(13, LOW); // B motor backwards	
	
	digitalWrite(8, LOW); // disable B motor brake
	analogWrite(11, speed);

	delay(1000);

	digitalWrite(8, HIGH); // enable motor B brake

}

void motor_both_cb(const geometry_msgs::Twist& msg) {

	uint8_t speed = (uint8_t) (255 * abs(msg.linear.x));
	
	#ifdef ENABLE_LOG
	char logoutput[MAX_LOG_LEN];
	snprintf(logoutput,MAX_LOG_LEN,"Both motors moving, speed: %d",speed);
	nh.loginfo(logoutput);
	#endif

	if (msg.linear.x > 0) {
		digitalWrite(13, HIGH); // B motor forward
		digitalWrite(12, LOW); // A motor forward
	}
	else {
		digitalWrite(13, LOW); // B motor backwards
		digitalWrite(12, HIGH); // A motor forward
	}

	
	digitalWrite(9, LOW); // disable A motor brake
	digitalWrite(8,LOW); // disable B motor brake
	
	analogWrite(11, speed);
	analogWrite(3,speed);

	delay(1000);

	digitalWrite(9, HIGH); // enable motor A brake
	digitalWrite(8, HIGH); // enable motor B brake
}

void servo_upper_cb(const geometry_msgs::Twist& msg) {

	int angle = (int) (360*msg.angular.x);

	#ifdef ENABLE_LOG
	char logoutput[MAX_LOG_LEN];
	snprintf(logoutput,MAX_LOG_LEN,"Upper servo moving, angle: %d",angle);
	nh.loginfo(logoutput);
	#endif

	servoA.write(angle);

}

void servo_lower_cb(const geometry_msgs::Twist& msg) {

	int angle = (int) (360*msg.angular.x);
	
	#ifdef ENABLE_LOG
	char logoutput[MAX_LOG_LEN];
	snprintf(logoutput,MAX_LOG_LEN, "Lower servo moving, angle: %d", angle);
	nh.loginfo(logoutput);
	#endif

	servoB.write(angle);

}
