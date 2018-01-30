package org.usfirst.frc.team159.robot.subsystems;

import org.usfirst.frc.team159.robot.RobotMap;
import org.usfirst.frc.team159.robot.commands.DriveWithJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 *
 */
public class DriveTrain extends Subsystem implements MotorSafety {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private WPI_TalonSRX frontLeft;
	private WPI_TalonSRX frontRight;
	private WPI_TalonSRX backLeft;
	private WPI_TalonSRX backRight;
	private static final double wheelDiameter = 6.1; //4.25
	private static final double gearRatio = 1; //(38/22)*3
	private static final double encoderTicks = 1024; //900
	private static final double encoderEdges = 4;
	public static final double ticksPerRevolution = gearRatio * encoderTicks * encoderEdges;
	public static final double feetPerRev = Math.PI * wheelDiameter / 12.0;
	public static final double ticksPerFoot = ticksPerRevolution / feetPerRev;
	private boolean inLowGear = false;
	private MotorSafetyHelper safetyHelper = new MotorSafetyHelper(this);
	private DoubleSolenoid gearPneumatic;
	ADXRS450_Gyro gyro;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new DriveWithJoystick());
	}

	public DriveTrain() {
		super();
		frontLeft = new WPI_TalonSRX(RobotMap.FRONT_LEFT);
		frontRight = new WPI_TalonSRX(RobotMap.FRONT_RIGHT);
		backLeft = new WPI_TalonSRX(RobotMap.BACK_LEFT);
		backRight = new WPI_TalonSRX(RobotMap.BACK_RIGHT);

		frontRight.configVelocityMeasurementWindow(4, 10);
		backLeft.configVelocityMeasurementWindow(4, 10);

		// frontRight.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms,
		// 10);
		// backLeft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms, 10);

		// frontRight.setStatusFramePeriod(StatusFrameEnhanced frame, 10 , 10)
		frontRight.set(ControlMode.PercentOutput, 0);
		backLeft.set(ControlMode.PercentOutput, 0);

		frontLeft.set(ControlMode.Follower, RobotMap.BACK_LEFT);
		backRight.set(ControlMode.Follower, RobotMap.FRONT_RIGHT);
		// frontRight.configEncoderCodesPerRev(ticks_per_foot);
		// backLeft.configEncoderCodesPerRev(ticks_per_foot);

		frontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		backLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		// frontRight.enableLimitSwitch(false, false);
		// backLeft.enableLimitSwitch(false, false);
		gearPneumatic = new DoubleSolenoid(RobotMap.GEARSHIFT_ID, 0, 1);

		gyro = new ADXRS450_Gyro();
		reset();
	}

	public void enable() {
		frontRight.set(ControlMode.PercentOutput, 0);
		backLeft.set(ControlMode.PercentOutput, 0);
		log();
	}

	public void reset() {
		log();
		frontRight.setStatusFramePeriod(com.ctre.phoenix.motorcontrol.StatusFrameEnhanced.Status_3_Quadrature, 4, 10);
		backLeft.setStatusFramePeriod(com.ctre.phoenix.motorcontrol.StatusFrameEnhanced.Status_3_Quadrature, 4, 10);
		// int s1=
		// frontRight.getStatusFramePeriod(com.ctre.phoenix.motorcontrol.StatusFrameEnhanced.Status_1_General,
		// 10);
		// int s2=
		// frontRight.getStatusFramePeriod(com.ctre.phoenix.motorcontrol.StatusFrameEnhanced.Status_3_Quadrature,
		// 10);
		// System.out.printf("general=%d quad=%d\n", s1, s2);

		// frontRight.reset();
		// backLeft.reset();
		backLeft.getSensorCollection().setQuadraturePosition(0, 5);
		frontRight.getSensorCollection().setQuadraturePosition(0, 5);
		gyro.reset();

		setLowGear();

	}

	public void disable() {
		frontRight.disable();
		backLeft.disable();
	}

	public void tankDrive(double left, double right) {
		setRaw(left, right);
		//backLeft.set(left);
		//frontRight.set(-right);
		//safetyHelper.feed();
		log();
	}

	double coerce(double min, double max, double value) {
		if (value < min) {
			value = min;
		} else if (value > max) {
			value = max;
		}
		return value;
	}

	public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {
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
				rightMotorOutput = moveValue - rotateValue;
				leftMotorOutput = Math.max(moveValue, rotateValue);
			} else {
				rightMotorOutput = Math.max(moveValue, -rotateValue);
				leftMotorOutput = moveValue + rotateValue;
			}
		} else {
			if (rotateValue > 0.0) {
				rightMotorOutput = -Math.max(-moveValue, rotateValue);
				leftMotorOutput = moveValue + rotateValue;
			} else {
				rightMotorOutput = moveValue - rotateValue;
				leftMotorOutput = -Math.max(-moveValue, -rotateValue);
			}
		}
		// Ramp values up
		// Make sure values are between -1 and 1
		leftMotorOutput = coerce(-1, 1, leftMotorOutput);
		rightMotorOutput = coerce(-1, 1, rightMotorOutput);
		/*
		 * System.out.printf("l:%d r:%d\n",
		 * frontRight.getSensorCollection().getQuadraturePosition(),
		 * -backLeft.getSensorCollection().getQuadraturePosition());
		 */
		setRaw(leftMotorOutput, rightMotorOutput);
		//backLeft.set(leftMotorOutput);
		//frontRight.set(-rightMotorOutput);
		//safetyHelper.feed();
		log();
	}

	@Override
	public void setExpiration(double timeout) {
		safetyHelper.setExpiration(timeout);
	}

	@Override
	public double getExpiration() {
		return safetyHelper.getExpiration();
	}

	@Override
	public boolean isAlive() {
		return safetyHelper.isAlive();
	}

	@Override
	public void stopMotor() {
		frontLeft.stopMotor();
		frontRight.stopMotor();
		backLeft.stopMotor();
		backRight.stopMotor();
		safetyHelper.feed();
	}

	@Override
	public void setSafetyEnabled(boolean enabled) {
		safetyHelper.setSafetyEnabled(enabled);
	}

	@Override
	public boolean isSafetyEnabled() {
		return safetyHelper.isSafetyEnabled();
	}

	@Override
	public String getDescription() {
		return "Robot Drive";
	}

	public double getDistance() {
		double d1 = getRightDistance();
		double d2 = getLeftDistance();
		double x = 0.5 * (d1 + d2);
		return x;
	}

	public double getRightDistance() {
		double ticks = -frontRight.getSensorCollection().getQuadraturePosition();
		return ticks / ticksPerFoot;
		// return frontRight.getPosition();
	}

	public double getLeftDistance() {
		double ticks = backLeft.getSensorCollection().getQuadraturePosition();
		return ticks / ticksPerFoot;
		// return -backLeft.getPosition();
	}

	public double getVelocity() {
		return (getLeftVelocity() + getRightVelocity()) / 2;
	}

	public double getLeftVelocity() {
		return (backLeft.getSensorCollection().getQuadratureVelocity() * 10) / ticksPerFoot;
	}

	public double getRightVelocity() {
		return (-frontRight.getSensorCollection().getQuadratureVelocity() * 10) / ticksPerFoot;
	}

	public void setLowGear() {
		if (!inLowGear) {
			gearPneumatic.set(DoubleSolenoid.Value.kReverse);
			inLowGear = true;
		}
	}

	public void setHighGear() {
		if (inLowGear) {
			gearPneumatic.set(DoubleSolenoid.Value.kForward);
			inLowGear = false;
		}
	}

	public void setRaw(double left, double right) {
		backLeft.set(left);
		frontRight.set(-right);
		safetyHelper.feed();
	}

	public boolean isInLowGear() {
		if (inLowGear) {
			return true;
		} else {
			return false;
		}
	}

	public double getHeading() {
		return gyro.getAngle();
	}

	void log() {
		SmartDashboard.putNumber("Heading", getHeading());
		SmartDashboard.putNumber("Left wheels", backLeft.get());
		SmartDashboard.putNumber("Right wheels", -frontRight.get());
		SmartDashboard.putNumber("Left distance", getLeftDistance());
		SmartDashboard.putNumber("Right distance", getRightDistance());
		SmartDashboard.putNumber("Velocity", getVelocity());
		SmartDashboard.putBoolean("Low Gear", inLowGear);
	}
}
