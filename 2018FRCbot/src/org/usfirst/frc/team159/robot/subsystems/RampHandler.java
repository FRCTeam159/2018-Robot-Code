package org.usfirst.frc.team159.robot.subsystems;

import org.usfirst.frc.team159.robot.commands.ManageRamp;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class RampHandler extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new ManageRamp());
	}
}
