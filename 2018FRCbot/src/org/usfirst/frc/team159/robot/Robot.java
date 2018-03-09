package org.usfirst.frc.team159.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team159.robot.commands.Autonomous;
import org.usfirst.frc.team159.robot.commands.Calibrate;
import org.usfirst.frc.team159.robot.commands.DrivePath;
import org.usfirst.frc.team159.robot.commands.DropGrabber;
import org.usfirst.frc.team159.robot.commands.PushCube;
import org.usfirst.frc.team159.robot.commands.SetElevatorHeight;
import org.usfirst.frc.team159.robot.subsystems.Cameras;
import org.usfirst.frc.team159.robot.subsystems.CubeHandler;
import org.usfirst.frc.team159.robot.subsystems.DriveTrain;
import org.usfirst.frc.team159.robot.subsystems.Elevator;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot implements RobotMap, Constants {

	public static Elevator elevator;
	public static CubeHandler cubeHandler;
	public static DriveTrain driveTrain;
	private static Cameras cameras;
	
	private static final boolean useDashboard = true;
	
	public static boolean preferScale = false;
	public static boolean forcedStraight = false;
	public static boolean oppositeSideAllowed = false;
	public static boolean useGyro = false;
	
	
	public static int targetObject = OBJECT_NONE;
	public static int targetSide = POSITION_ILLEGAL;
	public static int robotPosition = -1;
	
	public static int allBadOption = OTHER_SCALE;
	public static int allGoodOption = SAME_SCALE;


//	private static OI oi;
	
	public static final double powerScale = 0.6;

	private CommandGroup autonomousCommand;
	
	private SendableChooser<Integer> positionChooser = new SendableChooser<>();
	private SendableChooser<Integer> allBadChooser = new SendableChooser<>();
	private SendableChooser<Integer> allGoodChooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		driveTrain = new DriveTrain();
		elevator = new Elevator();
		cubeHandler = new CubeHandler();
		cameras = new Cameras();

//		oi = new OI();
		
		putValuesOnSmartDashboard();
	}
	
	@Override
	public void robotPeriodic() {
		SmartDashboard.putNumber("Elevator", Robot.elevator.getPosition());
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		getPreferences();
		getAutoTargets();
		
		autonomousCommand = new Autonomous();
		if(SmartDashboard.getBoolean("Calibrate", false)) {
			autonomousCommand.addSequential(new Calibrate());
		} else {
			autonomousCommand.addSequential(new DropGrabber());
			autonomousCommand.addSequential(new SetElevatorHeight(Elevator.SWITCH_HEIGHT, 5));
			autonomousCommand.addSequential(new DrivePath());
			if(targetObject == OBJECT_SCALE) {
				autonomousCommand.addSequential(new SetElevatorHeight(Elevator.SCALE_HEIGHT, 5));
			}
			if(targetObject != OBJECT_NONE) {
				autonomousCommand.addSequential(new PushCube(1));
			}
		}
		
		if (autonomousCommand != null) {
			autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		driveTrain.reset();
		
//		 This makes sure that the autonomous stops running when
//		 teleop starts running. If you want the autonomous to
//		 continue until interrupted by another command, remove
//		 this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {}
	
	private void putValuesOnSmartDashboard() {
		positionChooser.addObject("Left", POSITION_LEFT);
		positionChooser.addDefault("Center", POSITION_CENTER);
		positionChooser.addObject("Right", POSITION_RIGHT);
		
		allBadChooser.addDefault("Opposite Scale", OTHER_SCALE);
		allBadChooser.addObject("Opposite Switch", OTHER_SWITCH);
		allBadChooser.addObject("Go Straight", GO_STRAIGHT);

		allGoodChooser.addDefault("Prefer Scale", SAME_SCALE);
		allGoodChooser.addObject("Prefer Switch", SAME_SWITCH);

		SmartDashboard.putData("Position", positionChooser);
		SmartDashboard.putData("All Good Strategy", allGoodChooser);
		SmartDashboard.putData("All Bad Strategy", allBadChooser);
		
		SmartDashboard.putBoolean("Prefer Scale", false);
		SmartDashboard.putBoolean("Force Straight Path", false);
		SmartDashboard.putBoolean("Opposite Side Allowed", false);
		SmartDashboard.putNumber("Max Velocity", 1.5);
		SmartDashboard.putNumber("Max Acceleration", 22.25);
		SmartDashboard.putNumber("Max Jerk", 4);
		SmartDashboard.putNumber("GFACT", 2.0);
		SmartDashboard.putBoolean("Use Gyro", false);
		SmartDashboard.putString("Target", "Calculating");
		SmartDashboard.putString("FMS Data", "RLL");
//		SmartDashboard.putString("Position", "Center");

		SmartDashboard.putBoolean("Calibrate", false);
		SmartDashboard.putBoolean("Publish Path", false);
		SmartDashboard.putNumber("P", DrivePath.KP);
		
        SmartDashboard.putBoolean("Grabber Intake", false);
        SmartDashboard.putBoolean("Grabber Output", false);
        SmartDashboard.putBoolean("Grabber Arms", false);
	}
	
	private void getPreferences() {
		if(useDashboard) {
			getDashboardPreferences();
		} else {
			getSwitchPreferences();
		}
	}
	
	private void getDashboardPreferences() {
		preferScale = SmartDashboard.getBoolean("Prefer Scale", false);
		forcedStraight = SmartDashboard.getBoolean("Force Straight Path", false);
		oppositeSideAllowed = SmartDashboard.getBoolean("Opposite Side Allowed", false);
		useGyro = SmartDashboard.getBoolean("Use Gyro", false);
		allGoodOption = ((SendableChooser<Integer>) SmartDashboard.getData("All Good Strategy")).getSelected();
		allBadOption = ((SendableChooser<Integer>) SmartDashboard.getData("All Bad Strategy")).getSelected();
		Sendable position = SmartDashboard.getData("Position");
		SendableChooser<Integer> positionChooser = (SendableChooser<Integer>) position; 
		robotPosition = positionChooser.getSelected();
	}
	
	private void getSwitchPreferences() {
		//TODO complete this
		DigitalInput leftPosition = new DigitalInput(LEFT_POSITION_CHANNEL);
		DigitalInput centerPosition = new DigitalInput(CENTER_POSITION_CHANNEL);
		DigitalInput rightPosition = new DigitalInput(RIGHT_POSITION_CHANNEL);
		DigitalInput oppositeSide = new DigitalInput(ALLOW_OPPOSITE_CHANNEL);
		
		oppositeSideAllowed = oppositeSide.get();
		
		if(leftPosition.get()) {
			robotPosition = POSITION_LEFT;
		} else if(centerPosition.get()) {
			robotPosition = POSITION_CENTER;
		} else if(rightPosition.get()) {
			robotPosition = POSITION_RIGHT;
		}
	}
	
	private void getAutoTargets() {
		String gameMessage = DriverStation.getInstance().getGameSpecificMessage();
		String which_object[]= {"Switch","Scale","Straight"};
		String which_side[]= {"Center","Left","Right"};
		if (robotPosition == POSITION_CENTER) {
			targetObject = OBJECT_SWITCH;
			if (gameMessage.charAt(0) == 'R') {
				targetSide = POSITION_RIGHT;
			} else {
				targetSide = POSITION_LEFT;       
			}
		} else if (robotPosition == POSITION_RIGHT) {
			targetSide = POSITION_RIGHT;
			if (!forcedStraight) {
				if (gameMessage.charAt(1) == 'R' && gameMessage.charAt(0) == 'R') {
					if (preferScale) {
						targetObject = OBJECT_SCALE;
					} else {
						targetObject = OBJECT_SWITCH;
					}
				} else if (gameMessage.charAt(0) == 'R') {
					targetObject = OBJECT_SWITCH;
				} else if (gameMessage.charAt(1) == 'R') {
					targetObject = OBJECT_SCALE;
				} else if(allBadOption == OTHER_SCALE) { // LL
					targetObject = OBJECT_SCALE;
					targetSide = POSITION_LEFT;
				} else if(allBadOption == OTHER_SWITCH) { // LL
					targetObject = OBJECT_SWITCH;
					targetSide = POSITION_LEFT;
				} else {
					targetObject = OBJECT_NONE;
				}
			} else {
				targetObject = OBJECT_NONE;
			}
		} else if (robotPosition == POSITION_LEFT) {
			targetSide = POSITION_LEFT;
			if (forcedStraight) {
				if (gameMessage.charAt(1) == 'L' && gameMessage.charAt(0) == 'L') { //LL
					if (preferScale) {
						targetObject=OBJECT_SCALE;
					} else {
						targetObject=OBJECT_SWITCH;
					}
				} else if (gameMessage.charAt(0) == 'L') {  // LL LR
					targetObject=OBJECT_SWITCH;
				} else if (gameMessage.charAt(1) == 'L') { // RL
					targetObject=OBJECT_SCALE;
				} else if(allBadOption == OTHER_SCALE) { // RR
					targetObject=OBJECT_SCALE;
					targetSide=POSITION_RIGHT;
				} else if(allBadOption == OTHER_SWITCH) { // LL
					targetObject=OBJECT_SWITCH;
					targetSide=POSITION_RIGHT;
				} else {
					targetObject=OBJECT_NONE;
				}
			} else {
				targetObject=OBJECT_NONE;
			}
		}
		String targetString=which_side[targetSide]+"-"+which_object[targetObject];
		SmartDashboard.putString("Target", targetString);
	}
}
