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


    public static final int LEFT_INTAKE_MOTOR = 5;
    public static final int RIGHT_INTAKE_MOTOR = 6;
    public static final int ELEVATOR_MOTOR = 7;

    //Sensor IDs
    public static final int CUBE_DETECTOR_PING_CHANNEL = 0;
    public static final int CUBE_DETECTOR_ECHO_CHANNEL = 0;

    // Button IDs
    public static final int LEFT_JOYSTICK = 1;
    public static final int RIGHT_JOYSTICK = 4;
    public static final int LEFT_TRIGGER = 2;
    public static final int RIGHT_TRIGGER = 3;
    public static final int LOW_GEAR_BUTTON = 5;
    public static final int HIGH_GEAR_BUTTON = 6;
    public static final int INTAKE_BUTTON = 0;
    public static final int ARM_TOGGLE_BUTTON = 1;

    //other
    public static final int GEAR_SHIFTER_ID = 0;
    public static final int ARM_PISTON_ID = 0;
    //TODO put correct value

    //Constants
    public static final int ENCODER_WINDOW_SIZE = 4;
    public static final int ENCODER_STATUS_FRAME_PERIOD = 4;
    public static final int TIMEOUT = 10;
    public static final int ENCODER_TIMEOUT = 5;
    public static final int SOLENOID_FORWARD = 0;
    public static final int SOLENOID_REVERSE = 1;
}
