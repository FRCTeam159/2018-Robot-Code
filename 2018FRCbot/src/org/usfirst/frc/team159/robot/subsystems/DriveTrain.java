package org.usfirst.frc.team159.robot.subsystems;

import org.usfirst.frc.team159.robot.RobotMap;
import org.usfirst.frc.team159.robot.commands.DriveWithJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.command.Subsystem;
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
	public static double wheel_diameter=4.25;
	public static int ticks_per_rev=18654;
	public static double feet_per_rev=Math.PI*wheel_diameter/12.0;
	public static double ticks_per_foot= ticks_per_rev/feet_per_rev;
	private boolean inlowgear=false;
	private MotorSafetyHelper safetyHelper = new MotorSafetyHelper(this);
	private DoubleSolenoid gearPneumatic;
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new DriveWithJoystick());
	}
	
	public DriveTrain(){
		super();
		frontLeft = new WPI_TalonSRX(RobotMap.FRONTLEFT);
		frontRight = new WPI_TalonSRX(RobotMap.FRONTRIGHT);
		backLeft = new WPI_TalonSRX(RobotMap.BACKLEFT);
		backRight = new WPI_TalonSRX(RobotMap.BACKRIGHT);
		
		//frontRight.setStatusFramePeriod(StatusFrameEnhanced frame, 10 , 10)
		frontRight.set(ControlMode.PercentOutput,0);
		backLeft.set(ControlMode.PercentOutput,0);

		frontLeft.set(ControlMode.Follower,RobotMap.BACKLEFT);
		backRight.set(ControlMode.Follower,RobotMap.FRONTRIGHT);
		//frontRight.configEncoderCodesPerRev(ticks_per_foot);
		//backLeft.configEncoderCodesPerRev(ticks_per_foot);
		
		frontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		backLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		//frontRight.enableLimitSwitch(false, false);
		//backLeft.enableLimitSwitch(false, false);
		gearPneumatic = new DoubleSolenoid(RobotMap.GEARSHIFTID,0,1);
		reset();
	}
	
	public void enable(){
		frontRight.set(ControlMode.PercentOutput,0);
		backLeft.set(ControlMode.PercentOutput,0);
	}
	
	public void reset(){
		frontRight.setStatusFramePeriod(com.ctre.phoenix.motorcontrol.StatusFrameEnhanced.Status_3_Quadrature, 5,10);
		backLeft.setStatusFramePeriod(com.ctre.phoenix.motorcontrol.StatusFrameEnhanced.Status_3_Quadrature, 5,10);
	    //int s1=	frontRight.getStatusFramePeriod(com.ctre.phoenix.motorcontrol.StatusFrameEnhanced.Status_1_General, 10);
		//int s2=	frontRight.getStatusFramePeriod(com.ctre.phoenix.motorcontrol.StatusFrameEnhanced.Status_3_Quadrature, 10);
//		System.out.printf("general=%d quad=%d\n", s1, s2);

		//frontRight.reset();
		//backLeft.reset();
		backLeft.getSensorCollection().setQuadraturePosition(0, 5);
		frontRight.getSensorCollection().setQuadraturePosition(0, 5);
		setLowGear();
	}
	
	public void disable(){
		frontRight.disable();
		backLeft.disable();
	}
	
	public void tankDrive(double left, double right) {
		backLeft.set(-left);
		frontRight.set(right);
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
		/*System.out.printf("l:%d r:%d\n",
				frontRight.getSensorCollection().getQuadraturePosition(),
				-backLeft.getSensorCollection().getQuadraturePosition());
				*/
		backLeft.set(leftMotorOutput);
		frontRight.set(-rightMotorOutput);
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

	public double getRightDistance() {
		double ticks= frontRight.getSensorCollection().getQuadraturePosition();
		return ticks/ticks_per_foot;
		//return frontRight.getPosition();
	}

	public double getLeftDistance() {
		double ticks= -backLeft.getSensorCollection().getQuadraturePosition();
		return ticks/ticks_per_foot;
		//return -backLeft.getPosition();
	}
	public void setLowGear() {
		//cout << "SetLowGear"<<endl;

		if(!inlowgear){
			gearPneumatic.set(DoubleSolenoid.Value.kReverse);
			//cout << "Setting Low Gear"<<endl;
			inlowgear=true;
		}
	}

	public void setHighGear() {
		//cout << "SetHighGear"<<endl;
		if(inlowgear){
			gearPneumatic.set(DoubleSolenoid.Value.kForward);
			//cout << "Setting High Gear"<<endl;
			inlowgear=false;
		}
	}
	public boolean isInLowGear() {
		if(inlowgear){
			return true;
		} else {
			return false;
		}
	}
}
