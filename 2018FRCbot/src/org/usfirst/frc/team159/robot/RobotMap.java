package org.usfirst.frc.team159.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	//Controller IDs
	public static final int DRIVER = 0;
	public static final int OPERATOR = 1;
	
	//Motor IDs

	public static final int FRONT_LEFT = 3;
	public static final int BACK_LEFT = 4;

	public static final int FRONT_RIGHT = 2;	
	public static final int BACK_RIGHT = 1;


	public static final int LEFT_INTAKE_MOTOR=5;
	public static final int RIGHT_INTAKE_MOTOR=6;
	public static final int ELEVATOR_MOTOR=7;
	
	// Button IDs
	public static final int LEFT_JOYSTICK=1;
	public static final int RIGHT_JOYSTICK=4;
	public static final int LEFT_TRIGGER=2;
	public static final int RIGHT_TRIGGER=3;
	public static final int LOW_GEAR_BUTTON =5;
	public static final int HIGH_GEAR_BUTTON =6;
	
	//other
	public static final int GEARSHIFT_ID=0;
	
	//Constants
	public static final int ENCODER_LOOP_TIME = 5;
	public static final int TIMEOUT = 10;
	
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}
