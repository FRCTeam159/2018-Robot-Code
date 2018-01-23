package org.usfirst.frc.team159.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	//Controller IDs
	public static int DRIVER = 0;
	public static int OPERATOR = 1;
	//Motor IDs
	public static int FRONTLEFT = 3;
	public static int BACKLEFT = 4;

	public static int FRONTRIGHT = 2;	
	public static int BACKRIGHT = 1;
	
	public static int LEFTCUBEMOTOR=5;
	public static int RIGHTCUBEMOTOR=6;
	public static int ELEVATORMOTOR=7;
	// Button IDs
	public static int LEFTJOYSTICK=1;
	public static int RIGHTJOYSTICK=4;
	public static int LEFTTRIGGER=2;
	public static int RIGHTTRIGGER=3;
	public static int LOWGEAR_BUTTON=5;
	public static int HIGHGEAR_BUTTON=6;
	//other
	public static int GEARSHIFTID=0;
	//Constants
	public static final int ENCODERLOOPTIME = 5;
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
