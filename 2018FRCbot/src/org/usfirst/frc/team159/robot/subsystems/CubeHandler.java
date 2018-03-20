package org.usfirst.frc.team159.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;

import org.usfirst.frc.team159.robot.RobotMap;
import org.usfirst.frc.team159.robot.commands.CubeCommands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CubeHandler extends Subsystem implements MotorSafety {

    private WPI_TalonSRX leftIntakeMotor;
    private WPI_TalonSRX rightIntakeMotor;
	private DoubleSolenoid armPneumatic;

    private static final double intakePower = 1.0;
    private static final double outputPower = 1.0;
    private static final double holdPower = 0.2;

    private boolean armsOpen = false;
    
	private MotorSafetyHelper safetyHelper = new MotorSafetyHelper(this);

    public void initDefaultCommand() {
        setDefaultCommand(new CubeCommands());
    }

    public CubeHandler() {
        super();
        leftIntakeMotor = new WPI_TalonSRX(RobotMap.LEFT_INTAKE_MOTOR);
        rightIntakeMotor = new WPI_TalonSRX(RobotMap.RIGHT_INTAKE_MOTOR);

        armPneumatic = new DoubleSolenoid(RobotMap.ARM_PISTON_ID, RobotMap.ARM_FORWARD, RobotMap.ARM_REVERSE);
//        closeArms();

    }

    public void enable() {
        stop();
        leftIntakeMotor.set(ControlMode.PercentOutput, 0);
        rightIntakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public void disable() {
        stop();
        leftIntakeMotor.disable();
        rightIntakeMotor.disable();
    }

//	 Put methods for controlling this subsystem here. Call these from Commands.

    private void spinWheels(double power, boolean inwards) {
        if (inwards) {
            leftIntakeMotor.set(-power);
            rightIntakeMotor.set(power);
        } else {
            leftIntakeMotor.set(power);
            rightIntakeMotor.set(-power);
        }
		safetyHelper.feed();
    }
    
    public void hold() {
    	spinWheels(holdPower, true);
    }

    public void stop() {
    	spinWheels(0, false);
    }

//    public void toggleArms() {
//        if (armsOpen) {
//            closeArms();
//        } else {
//            openArms();
//        }
//    }

    public void openArms() {
    	armPneumatic.set(DoubleSolenoid.Value.kReverse);
        armsOpen = true;
    }

    public void closeArms() {
    	armPneumatic.set(DoubleSolenoid.Value.kForward);
        armsOpen = false;
    }

    public void intake() {
        spinWheels(intakePower, true);
    }

    public void output() {
        spinWheels(outputPower, false);
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
		leftIntakeMotor.stopMotor();
		rightIntakeMotor.stopMotor();
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
		return "Grabber";
	}
}
