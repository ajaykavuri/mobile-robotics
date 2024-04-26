
#include "main.h"
#include "lemlib/api.hpp"

//motor setup

//drive motors
pros::Motor rfm(18, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rmm(2, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rbm(8, pros::E_MOTOR_GEARSET_06, false);
pros::Motor lfm(20, pros::E_MOTOR_GEARSET_06, true);
pros::Motor lmm(17, pros::E_MOTOR_GEARSET_06, true);
pros::Motor lbm(10, pros::E_MOTOR_GEARSET_06, true);
//intake motor
pros::Motor intake(9, pros::E_MOTOR_GEARSET_18, true);
//puncher motors here
pros::Motor cata1(7, pros::E_MOTOR_GEARSET_36, false);

//pneumatics
pros::ADIDigitalOut pneum(1); //1-8 = "A"-"H"

//controller
pros::Controller master (CONTROLLER_MASTER);

//inertial
pros::Imu im(19);

//motor group setup
pros::MotorGroup rsm({rbm, rmm, rfm});
pros::MotorGroup lsm({lbm, lmm,lfm});

//drivetrain setup
lemlib::Drivetrain drivetrain {
	&lsm, //left motors
	&rsm, //right motors
	11.5, //track width
	3.25, //wheel diameter
	360.0, //rpm (36-60 gear ratio & 600 rpm motor)
	0.0 //chase power
};

//odometry sensor setup
lemlib::OdomSensors odomSensors {
	nullptr, //vertical tracking wheel 1
	nullptr, //vertical tracking wheel 2
	nullptr, //horizontal tracking wheel 1
	nullptr, //horizontal tracking wheel 2
	&im //intertial sensor
};

//linear controller (odom)
lemlib::ControllerSettings linearController { //(-0.1,0]
	15, //kP
	0, //kI
	30, //kD
	3, //anti-windup
	0.5, //small error
	100, //small error timeout
	1.5, //large error
	500, //large error timeout
	20 //slew rate
};

//angular controller (odom)
lemlib::ControllerSettings angularController {
	1.9, //kP
	0, //kI
	10, //kD
	3, //anti-windup
	0.5, //small error
	100, //small error timeout
	1.5, //large error
	500, //large error timeout
	0 //slew rate
};

//chassis setup
lemlib::Chassis chassi(drivetrain, linearController, angularController, odomSensors);

//brain screen
void screen(){
	while (true){
		//print current position info to brain
		//x and y don't work because inertial only works for turning
		lemlib::Pose pose = chassi.getPose();
		pros::lcd::print(0, "x: %f", pose.x);
		pros::lcd::print(1, "y: %f", pose.y);
		pros::lcd::print(2, "heading: %f", pose.theta);
		pros::lcd::print(3, "SkillsUSA Time!!");
		// master.print(1, 0, "%.2f", master.get_analog(ANALOG_LEFT_Y));
		// master.print(2, 0, "%.2f", master.get_analog(ANALOG_RIGHT_Y));
		pros::delay(10);
	}
}

//initialize
void initialize(){
	pros::lcd::initialize();
	chassi.calibrate();
	chassi.setPose(-43.033,-64.494,180); //starting position based on jerry path
	pros::Task screenTask(screen);
}

//auton
//load pure pursuit path
ASSET(skillsusa_txt);
ASSET(skillsusa2_txt);
ASSET(skillsusa3_txt);
ASSET(skillsusa4_txt);
ASSET(skillsusa5_txt);
void autonomous() {

	float med_speed = 90.0f; // this speed is for medium speed and better precision
	float full_speed = 127.0f; // this speed is best for pushing in triballs with maximum momentum
	float match_load_time = 32.0f; // we change this variable based on what we're testing
	float pure_pursuit_time = 15.0f; // this variable dictates the maximum time that pure pursuit can take

	// SKILLZ
	// go to matchload spot
	chassi.moveToPoint(-59.732,-49.971,2000,false,full_speed,false);
	// turn to face cata
	chassi.turnTo(37.403,-5.933,1000,false,full_speed,false);
	// move back to touch bar
	chassi.moveToPoint(-58.732,-49.971,2000,true,full_speed,false);


	// turn on cata and open wings
	pneum.set_value(true);
	cata1.move_velocity(300);
	// matchload for 32 seconds
	for (double i=0;i<=match_load_time;i+=0.01){
		master.print(1, 0, "%.2fs",match_load_time-i);
		pros::delay(10);
	}
	cata1.brake(); // stop cata
	pneum.set_value(false); // close wings
	chassi.setPose(-58.732, -49.971, 240); //reset position in case inertial sensor becomes uncalibrated due to cata loading

	// push balls into close goal
	chassi.turnTo(-64.863,-68,2000);
	chassi.moveToPoint(-64.863,-34.75,2000);

	//crossover to other side
	chassi.follow(skillsusa_txt, 15.0f,2800,false,false); // pure pursuit to cross over to other side
	pros::delay(100);
	chassi.setPose(34, -59, 270);
	pros::delay(100);
	chassi.follow(skillsusa2_txt, 15.0f, 1250, false, false); // navigate to side of far goal and push in balls
	pros::delay(1500);
	chassi.setPose(60, -20, 90);

	//go back and push triballs in again just in case
	chassi.moveToPoint(68, -20, 1000, true, med_speed);
	pros::delay(500);
	chassi.moveToPoint(60, -20, 1000, false, 110);
	pros::delay(500);
	chassi.moveToPoint(65, -20, 1000, true, med_speed);

	//set up for pushing in triballs from front
	chassi.turnTo(65, 100, 1000, true, med_speed);
	pros::delay(750);
	chassi.setPose(61.96, -36.517, 90);
	chassi.follow(skillsusa3_txt, 15.0f, 2000, false, false); // navigate to front of goal
	pros::delay(2000);
	chassi.turnTo(-144, -15.688, 500, true, med_speed);
	pros::delay(100);
	chassi.moveToPoint(48, -15.688, 1000, false, full_speed); // push in triballs from front
	pros::delay(200);
	chassi.follow(skillsusa4_txt, 15.0f, 2800, true, false); // navigate to bottom left corner to get ready to push in triballs in
	pros::delay(2000);
	chassi.turnTo(48, 10, 500, false, med_speed);
	pros::delay(100);
	pneum.set_value(true);
	pros::delay(200);
	chassi.follow(skillsusa5_txt, 15.0f, 2000, false, false); // gather triballs and push them in from front
	pros::delay(100);
	chassi.setPose(47.458, 8.042, 90);
	chassi.moveToPoint(72, 8.042, 1000, true, med_speed);
	pros::delay(200);
	chassi.moveToPoint(47.458, 8.042, 1000, false); // push in triballs again just in case
	
}

//driver control
bool pneumOut = false;
bool cataOn = false;

void opcontrol(){

	// run autonomous routine first to setup match loading quickly

	//go to matchload spot
  	chassi.moveToPoint(-59.732,-49.971,2000,false,127.0f,false);
	//turn to face cata
	chassi.turnTo(37.403,-1.933,1000,false,127.0f,false);
	//move back to touch bar
	chassi.moveToPoint(-58.732,-49.971,2000,true,127.0f,false);
	//turn on cata
	cataOn = true;
	cata1.move_velocity(100);

	while (true){

		float curve_value = 2.7f; // we custom tune this, depends on how fast our driver curves should be
		int full_vel = 128; // full velocity: makes sure that the robot moves this motor at full velocity

		chassi.tank(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y), curve_value);

		//intake
		if (master.get_digital(DIGITAL_L1)){
			intake.move_velocity(full_vel); // run intake forwards
		} else if (master.get_digital(DIGITAL_L2)) {
			intake.move_velocity(-1*full_vel); // else run intake backwards
		} else {
			intake.brake();
		}

		//puncher: pressing button once keeps it on until disabled
		if (master.get_digital_new_press(DIGITAL_R1)){
			if (!cataOn){
				cataOn = true;
				cata1.move_velocity(full_vel);
			}
		}

		// pressing this button once disables it until R1 is pressed
		if (master.get_digital_new_press(DIGITAL_R2)){
			if (cataOn){
				cataOn = false;
				cata1.brake();
			}
		}

		// pneumatics: set pneumatic value to opposite of what it is currently
		if (master.get_digital_new_press(DIGITAL_X)){
			pneumOut = !pneumOut;
			pneum.set_value(pneumOut);
			master.rumble("."); // user notification
		}

		pros::delay(10);
	}
}