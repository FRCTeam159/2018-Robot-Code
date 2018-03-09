package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.Robot;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class Autonomous extends CommandGroup {

    public Autonomous() {
    	requires(Robot.driveTrain);
//    	addSequential(new DropGrabber(1));
//        addSequential(new DrivePath());
    }
}
