package org.usfirst.frc.team159.robot.commands;

import java.util.ArrayList;

import org.usfirst.frc.team159.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Calibrate extends Command {
	private Timer runTimer = new Timer();
	private ArrayList<Double> rollingAverageList = new ArrayList<>();
	private double rollingAverageTotal = 0;
	private double speed = 0.7;
	
	private ArrayList<Double[]> velocityList = new ArrayList<>();
	
	private static final double runtime = 5;

    public Calibrate() {
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	rollingAverageList.clear();
    	Robot.driveTrain.reset();
    	runTimer.reset();
    	runTimer.start();
    	System.out.println("Calibrate.initialize()");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.setRaw(speed, speed);
    	double averageVelocity = getRollingAverage(feetToMeters(Robot.driveTrain.getVelocity()), 10);
    	velocityList.add(new Double[] {averageVelocity, runTimer.get()});
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return runTimer.get() >= runtime;
    }

    // Called once after isFinished returns true
    protected void end() {
    	double maxVelocity = 0;
    	double maxAcceleration = 0;
    	double maxJerk = 0;
    	
    	double lastAcceleration = 0;
    	double lastVelocity = 0;
    	double lastTime = 0;
    	
    	for(Double[] data : velocityList) {
    		double velocity = data[0];
    		double time = data[1];
    		double acceleration = getSlope(lastTime, lastVelocity, time, velocity)/(time - lastTime);
    		double jerk = getSlope(lastTime, lastAcceleration, time, acceleration)/(time - lastTime);;
    		
    		if(velocity > maxVelocity) {
    			maxVelocity = velocity;
    		}
    		if(acceleration > maxAcceleration) {
    			maxAcceleration = acceleration;
    		}
    		if(jerk > maxJerk) {
    			maxJerk = jerk;
    		}
    		
    		lastAcceleration = acceleration;
    		lastVelocity = velocity;
    		lastTime = time;
    	}
    	
    	Robot.driveTrain.disable();
    	System.out.println("Calibrate.end()\n");
    	System.out.println("Maximum velocity: " + maxVelocity);
    	System.out.println("Maximum acceleration: " + maxAcceleration);
    	System.out.println("Maximum jerk: " + maxJerk);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("Calibrate.interrupted()");
    }
    
    /*
     * Get the rolling average
     * 
     * Removes the oldest one and subtract it from the average if the list has reached its max size
     * Adds new velocity to list and to the total
     */
    private double getRollingAverage(double velocity, int bufferSize) {
    	if(rollingAverageList.size() >= bufferSize) {
    		rollingAverageTotal -= rollingAverageList.remove(0);
    	}
    	rollingAverageList.add(velocity);
    	rollingAverageTotal += velocity;
 
    	return rollingAverageTotal/rollingAverageList.size();
    }
    
    private double feetToMeters(double feet) {
    	return feet * 0.3048;
    }
    
    private double getSlope(double x1, double y1, double x2, double y2) {
    	return (y2-y1)/(x2-x1);
    }
}
