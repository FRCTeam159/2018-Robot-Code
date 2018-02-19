package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.OI;
import org.usfirst.frc.team159.robot.Robot;
import org.usfirst.frc.team159.robot.RobotMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CubeCommands extends Command {

    public CubeCommands() {
//         Use requires() here to declare subsystem dependencies eg. requires(chassis);
        requires(Robot.cubeHandler);
    }

    //     Called just before this Command runs the first time
    protected void initialize() {
        Robot.cubeHandler.enable();
    }

    //     Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Joystick stick = OI.operatorController;
    	boolean intakePressed = stick.getRawButton(RobotMap.INTAKE_BUTTON);
    	boolean outputPressed = stick.getRawButton(RobotMap.OUTPUT_BUTTON);
    	boolean armsOpened = stick.getRawButton(RobotMap.ARMS_OPEN_BUTTON);
    	boolean armsClosed = stick.getRawButton(RobotMap.ARMS_CLOSED_BUTTON);
    	
        if (intakePressed) {
            Robot.cubeHandler.intake();
        }
        if (outputPressed) {
        	Robot.cubeHandler.output();
        }
        if (!intakePressed && !outputPressed) {
        	Robot.cubeHandler.hold();
        }
        
        if (armsOpened) {
            Robot.cubeHandler.openArms();
        } else if (armsClosed) {
        	Robot.cubeHandler.closeArms();
        }
        
        SmartDashboard.putBoolean("Grabber Intake", intakePressed);
        SmartDashboard.putBoolean("Grabber Output", outputPressed);
        SmartDashboard.putBoolean("Grabber Arms", armsOpened);
    }

    //     Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    //     Called once after isFinished returns true
    protected void end() {
        Robot.cubeHandler.disable();
    }

    //     Called when another command which requires one or more of the same subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
