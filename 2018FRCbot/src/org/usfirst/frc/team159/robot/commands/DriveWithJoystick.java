package org.usfirst.frc.team159.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team159.robot.OI;
import org.usfirst.frc.team159.robot.Robot;
import org.usfirst.frc.team159.robot.RobotMap;

/**
 *
 */
public class DriveWithJoystick extends Command {
	public DriveWithJoystick() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Joystick stick = OI.driver_controller;
		// Get axis values
		double Left = stick.getRawAxis(RobotMap.LEFTTRIGGER);
    	double Right = stick.getRawAxis(RobotMap.RIGHTTRIGGER);	
    	boolean left_trigger=Left>0.5?true:false;
    	boolean right_trigger=Right>0.5?true:false;

		if (left_trigger || stick.getRawButton(RobotMap.LOWGEAR_BUTTON)){
			Robot.driveTrain.setLowGear();
		}
		else if(right_trigger || stick.getRawButton(RobotMap.HIGHGEAR_BUTTON)){
			Robot.driveTrain.setHighGear();
		}
    	double yAxis = stick.getRawAxis(RobotMap.LEFTJOYSTICK); // left stick - drive
    	double xAxis = -stick.getRawAxis(RobotMap.RIGHTJOYSTICK); // right stick - rotate
		Robot.driveTrain.arcadeDrive(yAxis, xAxis, true);
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
