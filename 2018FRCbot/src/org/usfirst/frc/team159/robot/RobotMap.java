package org.usfirst.frc.team159.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public interface  RobotMap {
//    Controller IDs
    public static final int DRIVER = 0;
    public static final int OPERATOR = 1;

//    Motor IDs
    public static final int FRONT_LEFT = 3;
    public static final int BACK_LEFT = 4;

    public static final int FRONT_RIGHT = 2;
    public static final int BACK_RIGHT = 1;


    public static final int LEFT_INTAKE_MOTOR = 6;
    public static final int RIGHT_INTAKE_MOTOR = 7;
    public static final int ELEVATOR_MOTOR = 5;

//    Sensor IDs
    public static final int CUBE_DETECTOR_ID = 0;

//     Button IDs
    public static final int LEFT_JOYSTICK = 1;
    public static final int RIGHT_JOYSTICK = 4;
    public static final int LEFT_TRIGGER = 2;
    public static final int RIGHT_TRIGGER = 3;
    public static final int LOW_GEAR_BUTTON = 5;
    public static final int HIGH_GEAR_BUTTON = 6;
    public static final int INTAKE_BUTTON = 1;
    public static final int OUTPUT_BUTTON = 2;
    
    public static final int ARMS_OPEN_BUTTON = 4;
    public static final int ARMS_CLOSED_BUTTON = 3;

    public static final int RESET_ELEVATOR_BUTTON = 5;
   // public static final int GO_TO_ELEVATOR_TOP_BUTTON = 6;
   public static final int ELEVATOR_GO_TO_SWITCH_BUTTON = 6;
   

//    other
    public static final int GEAR_SHIFTER_ID = 0;
    public static final int ARM_PISTON_ID = 0;

//    Constants
    public static final int ENCODER_WINDOW_SIZE = 4;
    public static final int ENCODER_STATUS_FRAME_PERIOD = 4;
    public static final int TIMEOUT = 10;
    public static final int ENCODER_TIMEOUT = 5;
    public static final int GEAR_SHIFTER_FORWARD = 0;
    public static final int GEAR_SHIFTER_REVERSE = 1;
    public static final int ARM_FORWARD = 2;
    public static final int ARM_REVERSE = 3;
    
    public static final int LEFT_POSITION = 0;
    public static final int CENTER_POSITION = 1;
    public static final int RIGHT_POSITION = 2;
    public static final int ILLEGAL_POSITION = 3;

//    DIO channels
    public static final int LEFT_POSITION_CHANNEL = 0;
    public static final int CENTER_POSITION_CHANNEL = 1;
    public static final int RIGHT_POSITION_CHANNEL = 2;
    
    public static final int ALLOW_OPPOSITE_CHANNEL = 3;
}
