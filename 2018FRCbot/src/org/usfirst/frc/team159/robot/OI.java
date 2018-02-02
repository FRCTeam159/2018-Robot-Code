package org.usfirst.frc.team159.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public static final Joystick driverController = new Joystick(RobotMap.DRIVER);
	public static final Joystick operatorController = new Joystick(RobotMap.OPERATOR);
	
//	public static Button elevatorDownButton = new JoystickButton(operatorController, 6);
//	public static Button elevatorUpButton = new JoystickButton(operatorController, 7);
//	public static Button lowGearButton = new JoystickButton(driverController, 4);
//	public static Button highGearButton = new JoystickButton(driverController, 5);
	public static Button armToggleButton = new JoystickButton(operatorController, RobotMap.ARM_TOGGLE_BUTTON);
	public static Button cubeIntakeButton = new JoystickButton(operatorController, RobotMap.INTAKE_BUTTON);
	
	
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
}