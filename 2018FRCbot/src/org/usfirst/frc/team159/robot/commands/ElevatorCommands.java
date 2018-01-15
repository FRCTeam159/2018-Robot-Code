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
		Joystick stick = OI.operator_controller;
    	double Left = -stick.getRawAxis(RobotMap.LEFTTRIGGER);
    	double Right = stick.getRawAxis(RobotMap.RIGHTTRIGGER);
    	double value = Right + Left;
		//System.out.printf("L=%f R=%f V=%f\n", Left, Right, value);	
		Robot.elevator.set(value);
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
    	System.out.println("Elevator.interrupted");
    	end();
    }
}
