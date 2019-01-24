#include "garry_uno_firmware.h"

// comment out to disable logging
//#define ENABLE_LOG

#define MAX_LOG_LEN 30
#define MOTOR_DELAY 1000

/* global variables */
Servo servoA;
Servo servoB;

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Bool> sub_led_right_eye("led_right_eye", &led_right_eye_cb);
ros::Subscriber<std_msgs::Bool> sub_led_left_eye("led_left_eye", &led_left_eye_cb);

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &cmd_vel_cb);

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
    nh.subscribe(sub_servo_upper);
    nh.subscribe(sub_servo_lower);
    nh.subscribe(sub_cmd_vel);
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

void cmd_vel_cb(const geometry_msgs::Twist& msg) {

    if(msg.linear.x != 0) {
        uint8_t speed = ((uint8_t) (abs(255 * msg.linear.x)));
        (msg.linear.x > 0) ? forward(speed) : backward(speed);
    }
    else if (msg.angular.z != 0) {
        uint8_t speed = ((uint8_t) (abs(255 * msg.angular.z)));
        (msg.angular.z > 0) ? turn_left(speed) : turn_right(speed);
    }
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
/* other functions */

void run_motors(const double speed) {

    digitalWrite(8, LOW); // disable B motor brake
    digitalWrite(9, LOW); // disable A motor brake
    analogWrite(3, speed);
    analogWrite(11, speed);

    delay(MOTOR_DELAY);

    digitalWrite(9, HIGH); //enable motor A brake
    digitalWrite(8, HIGH); // enable motor B brake
}

void turn_left(const double speed) {

    #ifdef ENABLE_LOG
    char logoutput[MAX_LOG_LEN];
    snprintf(logoutput,MAX_LOG_LEN,"Left motor moving, speed: %d",speed);
    nh.loginfo(logoutput);
    #endif

    digitalWrite(13, LOW); // B motor backwards
    digitalWrite(12, HIGH); // A motor forward

    run_motors(speed);
}

void turn_right(const double speed) {

    #ifdef ENABLE_LOG
    char logoutput[MAX_LOG_LEN];
    snprintf(logoutput,MAX_LOG_LEN,"Right motor moving, speed: %d",speed);
    nh.loginfo(logoutput);
    #endif

    digitalWrite(13, HIGH); // B motor forward
    digitalWrite(12, LOW); // A motor backward

    run_motors(speed);
}

void forward(const double speed) {

    #ifdef ENABLE_LOG
    char logoutput[MAX_LOG_LEN];
    snprintf(logoutput,MAX_LOG_LEN,"Both motors moving, speed: %d",speed);
    nh.loginfo(logoutput);
    #endif

    digitalWrite(13, HIGH); // B motor forward
    digitalWrite(12, HIGH); // A motor forward

    run_motors(speed);
}

void backward(const double speed) {

    digitalWrite(13, LOW); // B motor backwards
    digitalWrite(12, LOW); // A motor forward

    run_motors(speed);
}
