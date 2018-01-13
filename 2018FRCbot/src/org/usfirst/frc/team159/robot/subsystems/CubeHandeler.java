package org.usfirst.frc.team159.robot.subsystems;

import org.usfirst.frc.team159.robot.RobotMap;
import org.usfirst.frc.team159.robot.commands.CubeCommands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CubeHandeler extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private WPI_TalonSRX left;
	private WPI_TalonSRX right;
	public CubeHandeler() {
		super();
		left = new WPI_TalonSRX(RobotMap.LEFTCUBEMOTOR);
		right = new WPI_TalonSRX(RobotMap.RIGHTCUBEMOTOR);
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new CubeCommands());
    }
    public void spinWheels(double speed) {
    	
    }
}


