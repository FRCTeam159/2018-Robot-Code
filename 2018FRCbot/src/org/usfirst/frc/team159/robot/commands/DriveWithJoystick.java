package org.usfirst.frc.team159.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team159.robot.OI;
import org.usfirst.frc.team159.robot.Robot;
import org.usfirst.frc.team159.robot.RobotMap;

/**
 *
 */
public class DriveWithJoystick extends Command {
    private Timer timer = new Timer();
    private double lastVelocity = 0;
    private static final boolean debug = false;
    private static final double powerScale = 0.75;

    public DriveWithJoystick() {
        requires(Robot.driveTrain);
    }

    //	 Called just before this Command runs the first time
    @Override
    protected void initialize() {
        timer.start();
        timer.reset();
        Robot.elevator.setElevatorTarget(Robot.elevator.getPosition());
        SmartDashboard.putNumber("Move Exponent", SmartDashboard.getNumber("Move Exponent", 2));
        SmartDashboard.putNumber("Turn Exponent", SmartDashboard.getNumber("Turn Exponent", 2));
    }

    //	 Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Joystick stick = OI.driverController;
//		 Get axis values
//		double leftStick = stick.getRawAxis(RobotMap.LEFT_TRIGGER);
//    	double rightStick = stick.getRawAxis(RobotMap.RIGHT_TRIGGER);
//    	boolean leftTrigger = leftStick > 0.5;
//    	boolean rightTrigger = rightStick > 0.5;

        if (stick.getRawButton(RobotMap.LOW_GEAR_BUTTON)) {
            Robot.driveTrain.setLowGear();
        } else if (stick.getRawButton(RobotMap.HIGH_GEAR_BUTTON)) {
            Robot.driveTrain.setHighGear();
        }
        double moveAxis = -powerScale * stick.getRawAxis(RobotMap.LEFT_JOYSTICK); // left stick - drive
        double turnAxis = powerScale * stick.getRawAxis(RobotMap.RIGHT_JOYSTICK); // right stick - rotate
        double newVelocity = (Robot.driveTrain.getLeftVelocity()) * 0.3048; // in m/s
        double deltaVelocity = (newVelocity - lastVelocity) / 0.02;

        double moveExponent = SmartDashboard.getNumber("Move Exponent", 1);
        double turnExponent = SmartDashboard.getNumber("Turn Exponent", 1);

        Robot.driveTrain.arcadeDrive(moveAxis, turnAxis, moveExponent, turnExponent);

        if (debug) {
            System.out.format("%f %f %f %f %f %f %f %f %f\n",
                    timer.get(),
                    moveAxis,
                    turnAxis,
                    Robot.driveTrain.getLeftDistance(),
                    Robot.driveTrain.getRightDistance(),
                    Robot.driveTrain.getLeftVelocity(),
                    Robot.driveTrain.getRightVelocity(),
                    deltaVelocity,
                    Robot.driveTrain.getHeading());
        }
        lastVelocity = newVelocity;
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
