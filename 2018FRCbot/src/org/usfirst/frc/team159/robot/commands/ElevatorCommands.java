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

    private static final double maxSpeed = 40;
    private static final double cycleTime = 0.02;
    private static final double moveRate = cycleTime * maxSpeed;

    public ElevatorCommands() {
//		 Use requires() here to declare subsystem dependencies eg. requires(chassis);
        requires(Robot.elevator);
    }

    //	 Called just before this Command runs the first time
    protected void initialize() {
        Robot.elevator.setElevatorTarget(0);
        Robot.elevator.enable();
        printInitializeMessage();
    }

    //	 Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Joystick stick = OI.operatorController;
        double leftStick = 0.5 * (1 + stick.getRawAxis(RobotMap.LEFT_TRIGGER));
        double rightStick = 0.5 * (1 + stick.getRawAxis(RobotMap.RIGHT_TRIGGER));

        if (leftStick > 0) {
            decrementElevatorPosition(leftStick);
        }
        if (rightStick > 0) {
            incrementElevatorPosition(rightStick);
        }
    }

    //	 Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    //	 Called once after isFinished returns true
    protected void end() {
        Robot.elevator.reset();
        Robot.elevator.disable();
        printEndMessage();
    }

    //	 Called when another command which requires one or more of the same subsystems is scheduled to run
    protected void interrupted() {
        printInterruptedMessage();
        end();
    }

    private void incrementElevatorPosition(double value) {
        double position = Robot.elevator.getElevatorTarget();
        Robot.elevator.setElevatorTarget(position + (value * moveRate));
    }

    private void decrementElevatorPosition(double value) {
        double position = Robot.elevator.getElevatorTarget();
        Robot.elevator.setElevatorTarget(position - (value * moveRate));
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
