package org.usfirst.frc.team159.robot.subsystems;

import org.usfirst.frc.team159.robot.RobotMap;
import org.usfirst.frc.team159.robot.commands.DriveWithJoystick;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveTrain extends Subsystem implements MotorSafety {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	private CANTalon frontLeft;
	private CANTalon frontRight;
	private CANTalon backLeft;
	private CANTalon backRight;
	
	private MotorSafetyHelper safetyHelper = new MotorSafetyHelper(this);

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new DriveWithJoystick());
	}
	
	public DriveTrain(){
		super();
		frontLeft = new CANTalon(RobotMap.FRONTLEFT);
		frontRight = new CANTalon(RobotMap.FRONTRIGHT);
		backLeft = new CANTalon(RobotMap.BACKLEFT);
		backRight = new CANTalon(RobotMap.BACKRIGHT);
		
		frontRight.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		backLeft.changeControlMode(CANTalon.TalonControlMode.PercentVbus);

		frontLeft.changeControlMode(CANTalon.TalonControlMode.Follower);
		backRight.changeControlMode(CANTalon.TalonControlMode.Follower);
	}
	
	public void enable(){
		frontRight.enable();
		backLeft.enable();
	}
	
	public void tankDrive(double left, double right) {
		backLeft.set(-left);
		frontRight.set(-right);
		backRight.set(RobotMap.FRONTRIGHT);
		frontLeft.set(RobotMap.BACKLEFT);

		safetyHelper.feed();
	}
	
	double coerce(double min, double max, double x) {
		if (x < min)
			x = min;
		else if (x > max)
			x = max;
		return x;
	}
	
	public void arcadeDrive(double moveValue, double rotateValue,
			boolean squaredInputs) {
		double leftMotorOutput;
		double rightMotorOutput;

		if (squaredInputs) {
			// square the inputs (while preserving the sign) to increase fine control
			// while permitting full power
			if (moveValue >= 0.0) {
				moveValue = (moveValue * moveValue);
			} else {
				moveValue = -(moveValue * moveValue);
			}
			if (rotateValue >= 0.0) {
				rotateValue = (rotateValue * rotateValue);
			} else {
				rotateValue = -(rotateValue * rotateValue);
			}
		}

		if (moveValue > 0.0) {
			if (rotateValue > 0.0) {
				leftMotorOutput = moveValue - rotateValue;
				rightMotorOutput = Math.max(moveValue, rotateValue);
			} else {
				leftMotorOutput = Math.max(moveValue, -rotateValue);
				rightMotorOutput = moveValue + rotateValue;
			}
		} else {
			if (rotateValue > 0.0) {
				leftMotorOutput = -Math.max(-moveValue, rotateValue);
				rightMotorOutput = moveValue + rotateValue;
			} else {
				leftMotorOutput = moveValue - rotateValue;
				rightMotorOutput = -Math.max(-moveValue, -rotateValue);
			}
		}
		// Ramp values up
		// Make sure values are between -1 and 1
		leftMotorOutput  = coerce(-1, 1, leftMotorOutput);
		rightMotorOutput = coerce(-1, 1, rightMotorOutput);
		backLeft.set(leftMotorOutput);
		frontRight.set(-rightMotorOutput);
		backRight.set(RobotMap.FRONTRIGHT);
		frontLeft.set(RobotMap.BACKLEFT);
		safetyHelper.feed();
	}

	@Override
	public void setExpiration(double timeout) {
		// TODO Auto-generated method stub
		safetyHelper.setExpiration(timeout);
	}

	@Override
	public double getExpiration() {
		// TODO Auto-generated method stub
		return safetyHelper.getExpiration();
	}

	@Override
	public boolean isAlive() {
		// TODO Auto-generated method stub
		return safetyHelper.isAlive();
	}

	@Override
	public void stopMotor() {
		// TODO Auto-generated method stub
		frontLeft.stopMotor();
		frontRight.stopMotor();
		backLeft.stopMotor();
		backRight.stopMotor();
		safetyHelper.feed();
	}

	@Override
	public void setSafetyEnabled(boolean enabled) {
		// TODO Auto-generated method stub
		safetyHelper.setSafetyEnabled(enabled);
	}

	@Override
	public boolean isSafetyEnabled() {
		// TODO Auto-generated method stub
		return safetyHelper.isSafetyEnabled();
	}

	@Override
	public String getDescription() {
		// TODO Auto-generated method stub
		return "Robot Drive";
	}

	public double getDistance() {
		double d1=getRightDistance();
		double d2=getLeftDistance();
		double x=0.5*(d1+d2);
		return x;
	}

	private double getRightDistance() {
		return frontRight.getPosition();
	}

	private double getLeftDistance() {
		return -backLeft.getPosition();
	}
}
