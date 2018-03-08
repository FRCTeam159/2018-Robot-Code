package org.usfirst.frc.team159.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team159.robot.commands.Autonomous;
import org.usfirst.frc.team159.robot.commands.Calibrate;
import org.usfirst.frc.team159.robot.commands.DrivePath;
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
public class Robot extends IterativeRobot implements RobotMap {

	public static Elevator elevator;
	public static CubeHandler cubeHandler;
	public static DriveTrain driveTrain;
	private static Cameras cameras;
	
	private static final boolean useDashboard = true;
	
	public static boolean preferScale = false;
	public static boolean forcedStraight = false;
	public static boolean oppositeSideAllowed = false;
	public static boolean useGyro = false;
	
	public static int robotPosition = -1;


//	private static OI oi;
	
	public static final double powerScale = 0.6;

	private Command autonomousCommand;
	private SendableChooser<Integer> chooser = new SendableChooser<>();

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
		if(SmartDashboard.getBoolean("Calibrate", false)) {
			autonomousCommand = new Calibrate();
		} else {
			autonomousCommand = new Autonomous();
		}
		
		getPreferences();
		driveTrain.reset();
		
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
		chooser.addObject("Left", 0);
		chooser.addDefault("Center", 1);
		chooser.addObject("Right", 2);

		SmartDashboard.putData("Position", chooser);
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
		Sendable position = SmartDashboard.getData("Position");
		if(position != null) {
			SendableChooser<Integer> positionChooser = (SendableChooser<Integer>) position; 
			robotPosition = positionChooser.getSelected();
		}
	}
	
	private void getSwitchPreferences() {
		//TODO complete this
		DigitalInput leftPosition = new DigitalInput(LEFT_POSITION_CHANNEL);
		DigitalInput centerPosition = new DigitalInput(CENTER_POSITION_CHANNEL);
		DigitalInput rightPosition = new DigitalInput(RIGHT_POSITION_CHANNEL);
		DigitalInput oppositeSide = new DigitalInput(ALLOW_OPPOSITE_CHANNEL);
		
		oppositeSideAllowed = oppositeSide.get();
		
		if(leftPosition.get()) {
			robotPosition = LEFT_POSITION;
		} else if(centerPosition.get()) {
			robotPosition = CENTER_POSITION;
		} else if(rightPosition.get()) {
			robotPosition = RIGHT_POSITION;
		}
	}
}
