package org.usfirst.frc.team159.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Cameras extends Subsystem {

	public void initDefaultCommand() {
		// Set the default command for this subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public Cameras() {
		//CameraServer server = CameraServer.getInstance();
		//UsbCamera driverCamera = server.startAutomaticCapture("Driver", 0);
		//UsbCamera elevatorCamera = server.startAutomaticCapture("Elevator", 1);
		//driverCamera.setFPS(10);
		//elevatorCamera.setFPS(10);
	}
}
