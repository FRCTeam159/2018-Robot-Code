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
	Timer timer;
	private double lastVelocity = 0;
	private boolean debug = false;
	
	public DriveWithJoystick() {
		timer = new Timer();
		// Use requires() here to declare subsystem dependencies
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
		double Left = stick.getRawAxis(RobotMap.LEFTTRIGGER);
    	double Right = stick.getRawAxis(RobotMap.RIGHTTRIGGER);	
    	boolean leftTrigger = Left>0.5?true:false;
    	boolean rightTrigger = Right>0.5?true:false;

		if (leftTrigger || stick.getRawButton(RobotMap.LOWGEAR_BUTTON)){
			Robot.driveTrain.setLowGear();
		}
		else if(rightTrigger || stick.getRawButton(RobotMap.HIGHGEAR_BUTTON)){
			Robot.driveTrain.setHighGear();
		}
    	double yAxis = stick.getRawAxis(RobotMap.LEFTJOYSTICK); // left stick - drive
    	double xAxis = -stick.getRawAxis(RobotMap.RIGHTJOYSTICK); // right stick - rotate
		Robot.driveTrain.arcadeDrive(yAxis, xAxis, true);
		double newVelocity = (Robot.driveTrain.getLeftVelocity() * 10) * 0.3048; // in m/s
		double deltaVelocity = (newVelocity - lastVelocity)/0.02;
		if(debug)
		System.out.format("%f %f %f %f %f %f %f\n", 
				timer.get(),
				Robot.driveTrain.getLeftDistance(), 
				Robot.driveTrain.getRightDistance(), 
				Robot.driveTrain.getLeftVelocity(),
				Robot.driveTrain.getRightVelocity(),
				deltaVelocity,
				Robot.driveTrain.getHeading());
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
