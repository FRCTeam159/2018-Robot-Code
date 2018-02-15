package org.usfirst.frc.team159.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team159.robot.RobotMap;
import org.usfirst.frc.team159.robot.commands.ElevatorCommands;

/**
 *
 */
public class Elevator extends Subsystem implements PIDSource, PIDOutput {
    private WPI_TalonSRX elevatorMotor;

    private static final double P = 0.2;
    private static final double I = 0.0;
    private static final double D = 0.1;
    private static final double F = 0.0;

    private static final double WHEEL_DIAMETER = 1;
    private static final int ENCODER_EDGES = 4;
    private static final int ENCODER_TICKS = 1024;
    private static final double GEAR_RATIO = 1;
    private static final double TICKS_PER_REVOLUTION = GEAR_RATIO * ENCODER_TICKS * ENCODER_EDGES;
    private static final double FEET_PER_REV = Math.PI * WHEEL_DIAMETER / 12.0;
    private static final double TICKS_PER_FOOT = TICKS_PER_REVOLUTION / FEET_PER_REV;

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
        pidController = new PIDController(P, I, D, F, this, this);
        pidController.setOutputRange(-1, 1);
        pidController.disable();
    }

//	 Put methods for controlling this subsystem here. Call these from Commands.

    private double getPosition() {
        return metersToInches(elevatorMotor.getSensorCollection().getQuadraturePosition() / TICKS_PER_FOOT);
    }

    public void setElevatorTarget(double value) {
        value = value < 0 ? 0 : value;
        elevatorTarget = value;
        elevatorMotor.set(ControlMode.Position, elevatorTarget);
        SmartDashboard.putNumber("Elevator", Math.round(value));
    }

    public double getElevatorTarget() {
        return elevatorTarget;
    }

    public void reset() {
        pidController.disable();
        elevatorTarget = 0;
        pidController.reset();
        pidController.setSetpoint(elevatorTarget);
        pidController.disable();
    }

    public void enable() {
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
}