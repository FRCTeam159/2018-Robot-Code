package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.OI;
import org.usfirst.frc.team159.robot.Robot;
import org.usfirst.frc.team159.robot.RobotMap;
import org.usfirst.frc.team159.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ElevatorCommands extends Command {
    
    private boolean goingToBottom = false;
    private boolean goingToTop = false;
    private boolean goingToSwitch = false;

    public ElevatorCommands() {
//		 Use requires() here to declare subsystem dependencies eg. requires(chassis);
        requires(Robot.elevator);
    }

    //	 Called just before this Command runs the first time
    protected void initialize() {
//        Robot.elevator.setElevatorTarget(0);
        Robot.elevator.enable();
        printInitializeMessage();
    }

    //	 Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Joystick stick = OI.operatorController;
        double leftStick = stick.getRawAxis(RobotMap.LEFT_TRIGGER);
        double rightStick = stick.getRawAxis(RobotMap.RIGHT_TRIGGER);
        
        boolean goToZeroPressed = stick.getRawButton(RobotMap.RESET_ELEVATOR_BUTTON);
        boolean goToSwitchPressed = stick.getRawButton(RobotMap.ELEVATOR_GO_TO_SWITCH_BUTTON);
        
        if (goToZeroPressed) {
            Robot.elevator.setElevatorTarget(0);
            goingToBottom = true;
        }
//        if(OI.elevatorGoToTopButton.get()) {
//        	Robot.elevator.setElevatorTarget(Elevator.MAX_HEIGHT);
//        	goingToTop = true;
//        }
        if(goToSwitchPressed) {
        	Robot.elevator.setElevatorTarget(Elevator.START_HEIGHT);
        	goingToSwitch = true;
        }
        
        if(goingToBottom && Robot.elevator.getPosition() < Elevator.MOVE_RATE) {
        	goingToBottom = false;
        }
        if(goingToTop && Robot.elevator.getPosition() > Elevator.MAX_HEIGHT - Elevator.MOVE_RATE) {
        	goingToTop = false;
        }
        if(goingToSwitch && Robot.elevator.getPosition() > Elevator.SWITCH_HEIGHT - Elevator.MOVE_RATE && Robot.elevator.getPosition() < Elevator.SWITCH_HEIGHT + Elevator.MOVE_RATE) {
        	goingToSwitch = false;
        }
        
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
//        Robot.elevator.reset();
        printEndMessage();
    }

    //	 Called when another command which requires one or more of the same subsystems is scheduled to run
    protected void interrupted() {
        printInterruptedMessage();
        end();
    }

    private void incrementElevatorPosition(double value) {
        double position;
        if(goingToBottom || goingToTop || goingToSwitch) {
        	position = Robot.elevator.getPosition();
        } else {
        	position = Robot.elevator.getElevatorTarget();
        }
        
        Robot.elevator.setElevatorTarget(position + (value * Elevator.MOVE_RATE));
        goingToBottom = false;
        goingToTop = false;
        goingToSwitch = false;
    }

    private void decrementElevatorPosition(double value) {
        double position = Robot.elevator.getElevatorTarget();
        if(goingToBottom || goingToTop || goingToSwitch) {
        	position = Robot.elevator.getPosition();
        } else {
        	position = Robot.elevator.getElevatorTarget();
        }

        Robot.elevator.setElevatorTarget(position - (value * Elevator.MOVE_RATE));
        goingToBottom = false;
        goingToTop = false;
        goingToSwitch = false;
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
