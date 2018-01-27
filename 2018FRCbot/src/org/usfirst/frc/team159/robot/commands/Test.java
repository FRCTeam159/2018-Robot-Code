package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Test extends Command {
	Timer timer = new Timer();
    public Test() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	printInitializeMessage();
    	Robot.driveTrain.reset();
    	timer.start();
    	timer.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.tankDrive(0.0, 0.5);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return timer.get() >= 4.0;
    }

    // Called once after isFinished returns true
    protected void end() {
    	printEndMessage();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	
    }
    
    private void printInitializeMessage() {
    	System.out.println("Test.initialize()");
    }
    
    private void printEndMessage() {
    	System.out.println("Test.end()");
    }
}
