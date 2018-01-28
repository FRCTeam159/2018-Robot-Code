package org.usfirst.frc.team159.robot.subsystems;

import org.usfirst.frc.team159.robot.RobotMap;
import org.usfirst.frc.team159.robot.commands.ElevatorCommands;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Elevator extends Subsystem {
	private WPI_TalonSRX elevatorMotor;
	
	private static final int encoderEdges = 4;
	private static final int encoderTicks = 1024;
	private static final double gearRatio = 1;
	public static final double ticksPerRevolution = gearRatio * encoderTicks * encoderEdges;

	public Elevator() {
		super();
		elevatorMotor = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR);
		elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
	}
	
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new ElevatorCommands());
	}

	public void set(double value) {
		elevatorMotor.set(value);
	}

	public void reset() {

	}

}
