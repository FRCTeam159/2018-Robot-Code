package org.usfirst.frc.team159.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team159.robot.Robot;
import org.usfirst.frc.team159.robot.RobotMap;
import org.usfirst.frc.team159.robot.commands.ElevatorCommands;

/**
 *
 */
public class Elevator extends Subsystem implements PIDSource, PIDOutput, MotorSafety {
    private WPI_TalonSRX elevatorMotor;

    /* Known good PID settings (use as fallbacks)
     * 
     * P = 0.125
     * I = 0
     * D = 0.75
     * F = 0
     */
    
    private static final double P = 0.15;
    private static final double I = 0.0;
    private static final double D = 0.75;
    private static final double F = 0.0;

    private static final double WHEEL_DIAMETER = 1.75;
    private static final int ENCODER_EDGES = 4;
    private static final int ENCODER_TICKS = 1024;
    private static final double GEAR_RATIO = 1;
    private static final double TICKS_PER_REVOLUTION = GEAR_RATIO * ENCODER_TICKS * ENCODER_EDGES;
    private static final double INCHES_PER_REV = Math.PI * WHEEL_DIAMETER;
    private static final double TICKS_PER_INCH = TICKS_PER_REVOLUTION / INCHES_PER_REV;
    
    public static final double MAX_HEIGHT = 78;
    private static final double MIN_HEIGHT = 0;
    
	public static final double SWITCH_HEIGHT = 24;
	public static final double SCALE_HEIGHT = MAX_HEIGHT;
	public static final double START_HEIGHT = 4;
	
    private static final double MAX_SPEED = 60;
    private static final double CYCLE_TIME = 0.02;
    public static final double MOVE_RATE = CYCLE_TIME * MAX_SPEED;

    private MotorSafetyHelper safetyHelper = new MotorSafetyHelper(this);
    
    private PIDController pidController;
    private PIDSourceType pidType = PIDSourceType.kDisplacement;

    private double elevatorTarget = 0;

    public void initDefaultCommand() {
    	setDefaultCommand(new ElevatorCommands());
    }

    public Elevator() {
        super();
        elevatorMotor = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR);
        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, RobotMap.TIMEOUT);
//        elevatorMotor.configForwardLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyClosed, RobotMap.TIMEOUT);
//        elevatorMotor.configReverseLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, RobotMap.TIMEOUT);
//        elevatorMotor.overrideSoftLimitsEnable(false);
//        elevatorMotor.configForwardSoftLimitEnable(true, RobotMap.TIMEOUT);
//        elevatorMotor.configForwardSoftLimitThreshold(999999999, RobotMap.TIMEOUT);
        
        elevatorMotor.setStatusFramePeriod(com.ctre.phoenix.motorcontrol.StatusFrameEnhanced.Status_3_Quadrature, RobotMap.ENCODER_STATUS_FRAME_PERIOD, RobotMap.TIMEOUT);
        elevatorMotor.set(ControlMode.PercentOutput, 0);
//		Current limits set
    	//SmartDashboard.putNumber("Current limit", 20);
          pidController = new PIDController(P, I, D, F, this, this, 0.01);
        pidController.setOutputRange(-1.0, 1.0);
        reset();
//        pidController.disable();
    }

//	 Put methods for controlling this subsystem here. Call these from Commands.

    public double getPosition() {
    	double pos = (elevatorMotor.getSensorCollection().getQuadraturePosition() / TICKS_PER_INCH) * 2;
        SmartDashboard.putNumber("Elevator", pos);
        SmartDashboard.putNumber("ElevatorCurrent", elevatorMotor.getOutputCurrent());       
        return pos;
    }

    public void setElevatorTarget(double value) {
        value = value < MIN_HEIGHT ? MIN_HEIGHT : value;
        value = value > MAX_HEIGHT ? MAX_HEIGHT : value;
        elevatorTarget = value;
        pidController.setSetpoint(elevatorTarget);
    }
    
    public double getElevatorTarget() {
        return elevatorTarget;
    }
    
    public void set(double value) {
//    	System.out.println(value);
    	elevatorMotor.set(ControlMode.PercentOutput, value);
    	safetyHelper.feed();
    }

    public void reset() {
        elevatorTarget = 0;
        pidController.reset();
        pidController.enable();
        pidController.setSetpoint(elevatorTarget);
        elevatorMotor.set(ControlMode.PercentOutput, 0);
        elevatorMotor.getSensorCollection().setQuadraturePosition(0, RobotMap.ENCODER_TIMEOUT);
   // int ClValue = (int)SmartDashboard.getNumber("Current limit", 20);
    
        //elevatorMotor.configPeakCurrentLimit(0, RobotMap.TIMEOUT);
        //elevatorMotor.configContinuousCurrentLimit(ClValue, RobotMap.TIMEOUT);
    	//elevatorMotor.configPeakCurrentDuration(50, RobotMap.TIMEOUT);
    	//elevatorMotor.enableCurrentLimit(true);
    	

    }
    
    public boolean isAtTarget() {
    	return getPosition() > elevatorTarget - MOVE_RATE && getPosition() < elevatorTarget + MOVE_RATE;
    }

    public void enable() {
    	if(elevatorMotor.getSensorCollection().getQuadraturePosition() < 0) {
//    		elevatorMotor.getSensorCollection().setQuadraturePosition(0, RobotMap.TIMEOUT);
    	}
        pidController.enable();
    }

    public void disable() {
        pidController.disable();
    }

    private double metersToInches(double meters) {
        return (100 * meters) / 2.54;
    }
    
    @Override
    public void pidWrite(double value) {
        elevatorMotor.set(value);
    }

    @Override
    public double pidGet() {
        return getPosition();
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSourceType) {
        pidType = pidSourceType;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return pidType;
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
		elevatorMotor.stopMotor();
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
		return "Elevator";
	}
}