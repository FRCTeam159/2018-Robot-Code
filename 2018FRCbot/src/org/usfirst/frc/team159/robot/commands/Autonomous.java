package org.usfirst.frc.team159.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Autonomous extends CommandGroup {

    public Autonomous() {
    	addSequential(new DrivePath());
       //addSequential(new DriveStraight(5));
    }
}