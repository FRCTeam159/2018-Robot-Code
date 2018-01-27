package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraight extends Command implements PIDSource, PIDOutput {
	PIDController pid;
	PIDSourceType type = PIDSourceType.kDisplacement;
	double distance;
	double tolerance = 0.1;
	boolean last_ontarget = false;
	boolean started = false;
	static public boolean debug = true;
	static public double P = 0.25;
	static public double I = 0.002;
	static public double D = 0.0;
	static public double TOL = 0.05;
	static public double intime = 0.025;
	Timer timer = new Timer();;

	public DriveStraight(double distance) {
		this.distance = distance;
		pid = new PIDController(P, I, D, this, this);
		pid.setAbsoluteTolerance(TOL);
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.printf("divestraight.initialize distance = %f\n", distance);
		pid.reset();
		pid.setSetpoint(distance);
		pid.disable();
		Robot.driveTrain.enable();
		Robot.driveTrain.reset();
		started = false;
		timer.start();

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (started == false && timer.get() > intime && !pid.isEnabled()) {
			pid.reset();
			pid.enable();
			started = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (timer.get() > 3.0) {
			printFinishedMessage();
			return true;
		}
		return pid.onTarget();
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrain.disable();
		pid.disable();
		started = false;
		
		printEndMessage();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		type = pidSource;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return type;
	}

	@Override
	public double pidGet() {
		if (!started) {
			return 0;
		}
		if (debug) {
			double timeval = timer.get();
			System.out.printf("tm=%f l=%g r=%g d=%g\n", timeval * 1000, Robot.driveTrain.getLeftDistance(),
					Robot.driveTrain.getRightDistance(), Robot.driveTrain.getDistance());
		}
		return Robot.driveTrain.getDistance();
	}

	@Override
	public void pidWrite(double output) {
		if (started) {
			Robot.driveTrain.tankDrive(output, output);
		}
	}
	
	private void printEndMessage() {
		System.out.println("drivestraight.end");
	}
	
	private void printFinishedMessage() {
		System.out.println("timer expired");
	}
}
