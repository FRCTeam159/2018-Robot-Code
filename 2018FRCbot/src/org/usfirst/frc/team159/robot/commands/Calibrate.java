package org.usfirst.frc.team159.robot.commands;

import java.util.ArrayList;

import org.usfirst.frc.team159.Point;
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
	private double speed = 1;
	private int count = 0;
	
	private ArrayList<Double[]> velocityList = new ArrayList<>();
	
	private static final double runtime = 1;

    public Calibrate() {
    	requires(Robot.driveTrain);
    	runTimer.start();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	rollingAverageList.clear();
    	runTimer.reset();
    	Robot.driveTrain.reset();
    	Robot.driveTrain.enable();
    	printInitializeMessage();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(count == 0) {
    		//System.out.format("Skipping %f %f %f\n", runTimer.get(), feetToMeters(Robot.driveTrain.getDistance()), feetToMeters(Robot.driveTrain.getVelocity()));
    		runTimer.reset();
        	count++;
        	return;
    	}
    	if(runTimer.get() >= 0.25) {
    		Robot.driveTrain.setRaw(Robot.scale*speed, Robot.scale*speed);
    	}
    	double averageVelocity = getRollingAverage(feetToMeters(Robot.driveTrain.getVelocity()), 2);
    	velocityList.add(new Double[] {averageVelocity, runTimer.get()});
    	count++;
    	
    	//System.out.format("%f %f %f\n", runTimer.get(), feetToMeters(Robot.driveTrain.getDistance()), feetToMeters(Robot.driveTrain.getVelocity()));
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
    		double acceleration = getSlope(new Point(lastTime, lastVelocity), new Point(time, velocity));
    		double jerk = getSlope(new Point(lastTime, lastAcceleration), new Point(time, acceleration));
    		//double acceleration = (velocity - lastVelocity)/(time - lastTime);
    		//double jerk = (acceleration - lastAcceleration)/(time - lastTime);;
    		
    		if(velocity > maxVelocity) {
    			maxVelocity = velocity;
    		}
    		if(acceleration > maxAcceleration) {
    			maxAcceleration = acceleration;
    		}
    		if(jerk > maxJerk) {
    			maxJerk = jerk;
    		}
    		
    		System.out.format("%f %f %f\n", time, velocity, acceleration);
    		
    		lastAcceleration = acceleration;
    		lastVelocity = velocity;
    		lastTime = time;
    	}
    	
    	Robot.driveTrain.disable();
    	printEndMessage();
    	System.out.println("Maximum velocity: " + maxVelocity);
    	System.out.println("Maximum acceleration: " + maxAcceleration);
    	System.out.println("Maximum jerk: " + maxJerk);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	printInterruptedMessage();
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
    
    private double getSlope(Point point1, Point point2) {
    	return (point2.y - point1.y)/(point2.x - point1.x);
    }
    
    private void printInterruptedMessage() {
    	System.out.println("Calibrate.interrupted()");
    }
    
    private void printEndMessage() {
    	System.out.println("Calibrate.end()\n");
    }
    
    private void printInitializeMessage() {
    	System.out.println("Calibrate.initialize()");
    }
}
