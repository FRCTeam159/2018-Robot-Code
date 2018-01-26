package org.usfirst.frc.team159.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import org.usfirst.frc.team159.robot.commands.Autonomous;
import org.usfirst.frc.team159.robot.commands.Calibrate;
import org.usfirst.frc.team159.robot.commands.DriveWithJoystick;
import org.usfirst.frc.team159.robot.subsystems.Cameras;
import org.usfirst.frc.team159.robot.subsystems.CubeHandler;
import org.usfirst.frc.team159.robot.subsystems.DriveTrain;
import org.usfirst.frc.team159.robot.subsystems.Elevator;
import org.usfirst.frc.team159.robot.subsystems.RampHandler;

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
	public static RampHandler rampHandler;
	public static DriveTrain driveTrain;
	public static Cameras cameras;

	public static OI oi;
	
	public static double scale = 0.7;

	Command autonomousCommand;
	SendableChooser<Integer> chooser = new SendableChooser<Integer>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		driveTrain = new DriveTrain();
		elevator = new Elevator();
		cubeHandler = new CubeHandler();
		rampHandler = new RampHandler();
		cameras = new Cameras();

		oi = new OI();
		chooser.addObject("Left", new Integer(0));
		chooser.addDefault("Center", new Integer(1));
		chooser.addObject("Right", new Integer(2));
		SmartDashboard.putData("Position", chooser);
		SmartDashboard.putBoolean("Prefer Scale", true);
		SmartDashboard.putBoolean("Force Straight Path", false);
		SmartDashboard.putString("FMS Data", "???");
		SmartDashboard.putNumber("Max Velocity", 1.5);
		SmartDashboard.putNumber("Max Acceleration", 22.25);
		SmartDashboard.putNumber("Max Jerk", 30);
		SmartDashboard.putBoolean("Calibrate", false);
		SmartDashboard.putNumber("P", 1);
		//chooser.addDefault("Default Auto", new DriveWithJoystick());
		// chooser.addObject("My Auto", new MyAutoCommand());
		//SmartDashboard.putData("Auto mode", chooser);
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
		//autonomousCommand = chooser.getSelected();
		if(SmartDashboard.getBoolean("Calibrate", false)) {
			autonomousCommand = new Calibrate();
		} else {
			autonomousCommand = new Autonomous();
		}
		driveTrain.reset();
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
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
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		driveTrain.reset();
		elevator.reset();
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
	public void testPeriodic() {
		LiveWindow.run();
	}
}
