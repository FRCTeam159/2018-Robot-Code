package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.OI;
import org.usfirst.frc.team159.robot.Robot;

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
    	boolean intakePressed = OI.cubeIntakeButton.get();
    	boolean outputPressed = OI.cubeOutputButton.get();
    	boolean armsPressed = OI.armToggleButton.get();
    	
        if (intakePressed) {
            Robot.cubeHandler.intake();
        }
        if (outputPressed) {
        	Robot.cubeHandler.output();
        }
        if (armsPressed) {
            Robot.cubeHandler.toggleArms();
        }
        
        SmartDashboard.putBoolean("Grabber Intake", intakePressed);
        SmartDashboard.putBoolean("Grabber Output", outputPressed);
        SmartDashboard.putBoolean("Grabber Arms", armsPressed);
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
