package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.Robot;
import org.usfirst.frc.team159.robot.subsystems.DriveTrain;

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
	double distance;
	double tolerance = 0.1;
	boolean last_ontarget = false;
	static public boolean debug = true;
	PIDSourceType type = PIDSourceType.kDisplacement;
	static public double P = 0.25;
	static public double I = 0.002;
	static public double D = 0.0;
	static public double TOL = 0.05;
	static public double intime = 0.025;
	boolean started = false;
	Timer mytimer;

	public DriveStraight(double d) {
		mytimer = new Timer();
		distance = d;
		pid = new PIDController(P, I, D, this, this);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);
		pid.setAbsoluteTolerance(TOL);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.printf("divestraight.initialize d=%f\n", distance);
		pid.reset();
		pid.setSetpoint(distance);
		Robot.driveTrain.enable();
		Robot.driveTrain.reset();
		pid.disable();
		started = false;
		mytimer.start();

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (started == false && mytimer.get() > intime && !pid.isEnabled()) {
			pid.reset();
			pid.enable();
			started = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (mytimer.get() > 3.0) {
			System.out.println("timer expired");
			return true;
		}
		return pid.onTarget();
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("drivestraight.end");
		Robot.driveTrain.disable();
		pid.disable();
		started = false;
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
		if (!started)
			return 0;
		if (debug) {
			double timeval = mytimer.get();
			System.out.printf("tm=%f l=%g r=%g d=%g\n", timeval * 1000, Robot.driveTrain.getLeftDistance(),
					Robot.driveTrain.getRightDistance(), Robot.driveTrain.getDistance());
		}
		return Robot.driveTrain.getDistance();
	}

	@Override
	public void pidWrite(double output) {
		if (started)
			Robot.driveTrain.tankDrive(output, output);
	}
}