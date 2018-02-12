package org.usfirst.frc.team159.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
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
public class Robot extends IterativeRobot {

	public static Elevator elevator;
	public static CubeHandler cubeHandler;
	public static DriveTrain driveTrain;
	private static Cameras cameras;

//	private static OI oi;
	
	public static final double scale = 0.6;

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
		
		System.out.println(new SendableChooser<Integer>());
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

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		if(SmartDashboard.getBoolean("Calibrate", false)) {
			autonomousCommand = new Calibrate();
		} else {
			autonomousCommand = new Autonomous();
		}
		
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
		elevator.reset();
		
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
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
		SmartDashboard.putString("FMS Data", "RLL");
		SmartDashboard.putNumber("Max Velocity", 1.5);
		SmartDashboard.putNumber("Max Acceleration", 22.25);
		SmartDashboard.putNumber("Max Jerk", 4);		
		SmartDashboard.putNumber("GFACT", 2.0);
		SmartDashboard.putBoolean("Use Gyro", false);
		SmartDashboard.putString("Target", "Calculating");

		SmartDashboard.putBoolean("Calibrate", false);
		SmartDashboard.putBoolean("Publish Path", false);
		SmartDashboard.putNumber("P", DrivePath.KP);
	}
}
