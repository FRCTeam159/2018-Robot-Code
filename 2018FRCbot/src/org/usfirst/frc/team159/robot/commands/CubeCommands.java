package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.OI;
import org.usfirst.frc.team159.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CubeCommands extends Command {

    private boolean intakeLastPressed = false;
    private boolean armToggleLastPressed = false;

    public CubeCommands() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.cubeHandler);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.cubeHandler.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        boolean intakePressed = OI.cubeIntakeButton.get();
        boolean armTogglePressed = OI.armToggleButton.get();
        if(intakePressed != intakeLastPressed){
            doIntakeActions();
        }
        if(armTogglePressed != armToggleLastPressed){
            Robot.cubeHandler.toggleArms();
        }
        if(Robot.cubeHandler.cubeDetected() && Robot.cubeHandler.isIntakeStarted()){
            Robot.cubeHandler.stopIntake();
        }

        armToggleLastPressed = armTogglePressed;
        intakeLastPressed = intakePressed;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.cubeHandler.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }

    private void doIntakeActions(){
        if(Robot.cubeHandler.cubeDetected()){
            Robot.cubeHandler.startOutput();
        } else {
            Robot.cubeHandler.toggleIntake();
        }
    }
}
