package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.OI;
import org.usfirst.frc.team159.robot.Robot;
import org.usfirst.frc.team159.robot.RobotMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorCommands extends Command {

    public ElevatorCommands() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Elevator.initialize");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		Joystick stick = OI.driver_controller;
    	double Left = stick.getRawAxis(RobotMap.LEFTTRIGGER);
    	double Right = stick.getRawAxis(RobotMap.RIGHTTRIGGER);
		System.out.printf("L=%f R=%g", Left, Right);
    		
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("Elevator.end");

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
