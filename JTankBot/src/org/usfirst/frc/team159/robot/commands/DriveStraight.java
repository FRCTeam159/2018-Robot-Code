package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraight extends Command implements PIDSource, PIDOutput {
	PIDController pid;
	double distance;
	double tolerance=0.1;
	boolean last_ontarget=false;
	PIDSourceType type=PIDSourceType.kDisplacement;
	static public double P=1.0;
	static public double I=0.0;
	static public double D=0.0;
	static public double TOL=0.05;
    public DriveStraight(double d) {
    	distance=d;
    	pid=new PIDController(P,I,D,this,this);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		type= pidSource;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return type;
	}

	@Override
	public double pidGet() {
		return Robot.driveTrain.getDistance();
	}

	@Override
	public void pidWrite(double output) {
		Robot.driveTrain.tankDrive(output, output);		
	}
}
