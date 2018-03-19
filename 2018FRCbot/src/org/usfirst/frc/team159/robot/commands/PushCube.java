package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PushCube extends Command {
	
	private Timer timer = new Timer();
	
	private double time;

    public PushCube(double time) {
    	requires(Robot.cubeHandler);
    	
    	this.time = time;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.cubeHandler.output();
    	timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }
    
    protected boolean isFinished() {
    	return timer.get() >= time;
    }
    
    protected void end() {
    	Robot.cubeHandler.hold();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
