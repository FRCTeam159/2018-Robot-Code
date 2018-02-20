package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.Robot;
import org.usfirst.frc.team159.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DropGrabber extends Command {
	
	Timer timer = new Timer();

	double time = 0.5;
	
	public DropGrabber() {}
	
    public DropGrabber(double time) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.time = time;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.reset();
    	Robot.elevator.setElevatorTarget(Elevator.START_HEIGHT);
    	timer.reset();
    	timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return timer.get() > time;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
