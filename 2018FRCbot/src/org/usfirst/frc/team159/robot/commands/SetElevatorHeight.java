package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetElevatorHeight extends Command {

	private double height;
	private double timeout;
	
	private Timer timer = new Timer();
	
    public SetElevatorHeight(double height, double timeout) {
        requires(Robot.elevator);
        this.height = height;
        this.timeout = timeout;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.setElevatorTarget(height);
    	timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.elevator.isAtTarget() || timer.get() >= timeout;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.elevator.setElevatorTarget(Robot.elevator.getPosition());
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
