package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.Constants;
import org.usfirst.frc.team159.robot.Robot;
import org.usfirst.frc.team159.robot.RobotMap;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class SetGrabberState extends TimedCommand implements Constants {
    int state=HOLD;

    public SetGrabberState(int state, double timeout) {
        super(timeout);
        this.state=state;
        requires(Robot.cubeHandler);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
      System.out.println("SetGrabberState.initialize:"+state);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
      Robot.cubeHandler.setState(state);
    }

    // Called once after timeout
    protected void end() {
      System.out.println("SetGrabberState.end");
      Robot.cubeHandler.hold();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
      end();
    }
}
