package org.usfirst.frc.team159.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team159.robot.OI;
import org.usfirst.frc.team159.robot.Robot;
import org.usfirst.frc.team159.robot.RobotMap;

/**
 *
 */
public class DriveWithJoystick extends Command {
	Timer timer = new Timer();
	private double lastVelocity = 0;
	private boolean debug = false;
	private static final double powerScale = 0.75;
	
	public DriveWithJoystick() {
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		timer.start();
		timer.reset();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Joystick stick = OI.driverController;
		// Get axis values
		double leftStick = stick.getRawAxis(RobotMap.LEFTTRIGGER);
    	double rightStick = stick.getRawAxis(RobotMap.RIGHTTRIGGER);	
    	boolean leftTrigger = leftStick > 0.5 ? true : false;
    	boolean rightTrigger = rightStick > 0.5 ? true : false;

		if (leftTrigger || stick.getRawButton(RobotMap.LOWGEAR_BUTTON)){
			Robot.driveTrain.setLowGear();
		} else if(rightTrigger || stick.getRawButton(RobotMap.HIGHGEAR_BUTTON)){
			Robot.driveTrain.setHighGear();
		}
    	double moveAxis = -powerScale*stick.getRawAxis(RobotMap.LEFTJOYSTICK); // left stick - drive
    	double turnAxis = powerScale*stick.getRawAxis(RobotMap.RIGHTJOYSTICK); // right stick - rotate
		double newVelocity = (Robot.driveTrain.getLeftVelocity()) * 0.3048; // in m/s
		double deltaVelocity = (newVelocity - lastVelocity)/0.02;
		Robot.driveTrain.arcadeDrive(moveAxis, turnAxis, true);
		if(debug) {
		System.out.format("%f %f %f %f %f %f %f %f %f\n", 
				timer.get(),
				moveAxis,
				turnAxis,
				Robot.driveTrain.getLeftDistance(), 
				Robot.driveTrain.getRightDistance(), 
				Robot.driveTrain.getLeftVelocity(),
				Robot.driveTrain.getRightVelocity(),
				deltaVelocity,
				Robot.driveTrain.getHeading());
		}
		lastVelocity = newVelocity;
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {

	}
}
