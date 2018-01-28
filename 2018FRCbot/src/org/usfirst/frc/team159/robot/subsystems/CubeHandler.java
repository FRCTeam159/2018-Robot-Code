package org.usfirst.frc.team159.robot.subsystems;

import org.usfirst.frc.team159.robot.RobotMap;
import org.usfirst.frc.team159.robot.commands.CubeCommands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CubeHandler extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private WPI_TalonSRX leftIntakeMotor;
	private WPI_TalonSRX rightIntakeMotor;

	public CubeHandler() {
		super();
		leftIntakeMotor = new WPI_TalonSRX(RobotMap.LEFT_INTAKE_MOTOR);
		rightIntakeMotor = new WPI_TalonSRX(RobotMap.RIGHT_INTAKE_MOTOR);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new CubeCommands());
	}

	public void spinWheels(double speed) {

	}
}
