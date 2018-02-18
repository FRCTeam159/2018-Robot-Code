package org.usfirst.frc.team159.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.usfirst.frc.team159.robot.RobotMap;
import org.usfirst.frc.team159.robot.commands.CubeCommands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CubeHandler extends Subsystem {

    private WPI_TalonSRX leftIntakeMotor;
    private WPI_TalonSRX rightIntakeMotor;
//	private DoubleSolenoid armPneumatic;

    private static final double intakePower = 0.5;
    private static final double outputPower = 0.25;

    private boolean armsOpen = false;

    public void initDefaultCommand() {
//        setDefaultCommand(new CubeCommands());
    }

    public CubeHandler() {
        super();
//        leftIntakeMotor = new WPI_TalonSRX(RobotMap.LEFT_INTAKE_MOTOR);
//        rightIntakeMotor = new WPI_TalonSRX(RobotMap.RIGHT_INTAKE_MOTOR);

//		 commented out because it throws exceptions if the IDs are wrong
//		armPneumatic = new DoubleSolenoid(RobotMap.ARM_PISTON_ID, RobotMap.SOLENOID_FORWARD, RobotMap.SOLENOID_REVERSE);
    }

    public void enable() {
        stop();
        openArms();
    }

    public void disable() {
        stop();
        leftIntakeMotor.disable();
        rightIntakeMotor.disable();
    }

//	 Put methods for controlling this subsystem here. Call these from Commands.

    private void spinWheels(double power, boolean inwards) {
        if (inwards) {
            leftIntakeMotor.set(power);
            rightIntakeMotor.set(-power);
        } else {
            leftIntakeMotor.set(-power);
            rightIntakeMotor.set(power);
        }
    }

    public void stop() {
    	leftIntakeMotor.set(0);
        rightIntakeMotor.set(0);
    }

    public void toggleArms() {
        if (armsOpen) {
            closeArms();
        } else {
            openArms();
        }
    }

    private void openArms() {
//	    armPneumatic.set(DoubleSolenoid.Value.kReverse);
        armsOpen = true;
    }

    private void closeArms() {
//      armPneumatic.set(DoubleSolenoid.Value.kForward);
        armsOpen = false;
    }

    public void intake() {
        spinWheels(intakePower, true);
    }

    public void output() {
        spinWheels(outputPower, false);
    }
}
