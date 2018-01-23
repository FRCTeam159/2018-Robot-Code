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
<<<<<<< HEAD
	public static int FRONTLEFT = 3;
	public static int BACKLEFT = 4;

	public static int FRONTRIGHT = 2;	
	public static int BACKRIGHT = 1;
=======
	public static final int FRONTLEFT = 1;
	public static final int FRONTRIGHT = 4;
	public static final int BACKLEFT = 2;
	public static final int BACKRIGHT = 3;
	public static final int LEFTCUBEMOTOR=5;
	public static final int RIGHTCUBEMOTOR=6;
	public static final int ELEVATORMOTOR=7;
>>>>>>> e1f42478911208b0be8c23c746b10a2da86b64ca
	
	// Button IDs
	public static final int LEFTJOYSTICK=1;
	public static final int RIGHTJOYSTICK=4;
	public static final int LEFTTRIGGER=2;
	public static final int RIGHTTRIGGER=3;
	public static final int LOWGEAR_BUTTON=5;
	public static final int HIGHGEAR_BUTTON=6;
	
	//other
	public static final int GEARSHIFTID=0;
	
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
