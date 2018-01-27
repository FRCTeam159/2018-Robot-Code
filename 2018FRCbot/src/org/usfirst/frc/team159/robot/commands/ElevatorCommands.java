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
		printInitializeMessage();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Joystick stick = OI.operatorController;
		double leftStick = -stick.getRawAxis(RobotMap.LEFTTRIGGER);
		double rightStick = stick.getRawAxis(RobotMap.RIGHTTRIGGER);
		double value = rightStick + leftStick;
		// System.out.printf("L=%f R=%f V=%f\n", Left, Right, value);
		Robot.elevator.set(value);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		printEndMessage();

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		printInterruptedMessage();
		end();
	}
	
	private void printInitializeMessage() {
		System.out.println("Elevator.initialize");
	}
	
	private void printEndMessage() {
		System.out.println("Elevator.end");
	}
	
	private void printInterruptedMessage() {
		System.out.println("Elevator.interrupted");
	}
}
