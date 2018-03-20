package org.usfirst.frc.team159.robot;

public interface Constants {
	public static final int OBJECT_SWITCH = 0;
	public static final int OBJECT_SCALE = 1;
	public static final int OBJECT_NONE = 2;
	
    public static final int POSITION_LEFT = 0;
    public static final int POSITION_CENTER = 1;
    public static final int POSITION_RIGHT = 2;
    public static final int POSITION_ILLEGAL = 3;
    
//    public static final int OTHER_SCALE = 0;
//    public static final int OTHER_SWITCH = 1;
//    public static final int GO_STRAIGHT = 2;
//    
//    public static final int SAME_SCALE = 0;
//    public static final int SAME_SWITCH = 1;
    
    // strategy options
    //TODO change prefix to STRATEGY_OPTION
    public static final int STRATEGY_TWO_CUBES = 3;
    public static final int STRATEGY_OPPOSITE_SCALE = 2;
    public static final int STRATEGY_SAME_SIDE_SCALE = 1;
    public static final int STRATEGY_SAME_SIDE_SWITCH = 0;
    
    // actual strategies
    public static final int TARGET_CENTER_SWITCH = 0;
    public static final int TARGET_CENTER_TWO_CUBES = 1;
    public static final int TARGET_SIDE_TWO_CUBES = 2;
    public static final int TARGET_SAME_SCALE = 3;
    public static final int TARGET_SAME_SWITCH = 4;
    public static final int TARGET_OTHER_SCALE = 5;
    public static final int TARGET_GO_STRAIGHT = 6;
    
    // grabber states
    public static final int HOLD = 0;
    public static final int PUSH = 1;
    public static final int GRAB = 2;
    public static final int OPEN = 3;
    public static final int CLOSE = 4;

}
