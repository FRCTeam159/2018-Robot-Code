package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class SetElevator extends TimedCommand {
    double pos=0;
    public SetElevator(double value, double timeout) {
        super(timeout);
        pos=value;
        requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
      System.out.println("SetElevator.initialize");
      Robot.elevator.setElevatorTarget(pos);
      Robot.elevator.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
      Robot.cubeHandler.hold();
    }

    // Called repeatedly when this Command is scheduled to run
    protected boolean isFinished() {
    	return true;
//      return super.isFinished()|| Robot.elevator.atTarget();
    }

    // Called once after timeout
    protected void end() {
      System.out.println("SetElevator.end");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
      end();
    }
}
