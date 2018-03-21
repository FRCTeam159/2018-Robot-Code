package org.usfirst.frc.team159.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team159.robot.commands.AutoSelector;
import org.usfirst.frc.team159.robot.commands.Autonomous;
import org.usfirst.frc.team159.robot.commands.Calibrate;
import org.usfirst.frc.team159.robot.commands.DrivePath;
import org.usfirst.frc.team159.robot.commands.DropGrabber;
import org.usfirst.frc.team159.robot.commands.PushCube;
import org.usfirst.frc.team159.robot.commands.SetElevatorHeight;
import org.usfirst.frc.team159.robot.subsystems.Cameras;
import org.usfirst.frc.team159.robot.subsystems.CubeHandler;
import org.usfirst.frc.team159.robot.subsystems.DIOSwitches;
import org.usfirst.frc.team159.robot.subsystems.DriveTrain;
import org.usfirst.frc.team159.robot.subsystems.Elevator;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot implements RobotMap, Constants {

	public static Elevator elevator;
	public static CubeHandler cubeHandler;
	public static DriveTrain driveTrain;
	private static Cameras cameras;
	public static DIOSwitches DIOs;

	public static boolean useGyro = false;
	public static final boolean useHardware = true;

	// public static int targetObject = OBJECT_NONE;
	// public static int targetSide = POSITION_ILLEGAL;
	public static int robotPosition = -1;
	public static String fmsData = "LLL";

	public static final double DRIVEPATH_MAX_VELOCITY = 2.6218;
	public static final double DRIVEPATH_MAX_ACCELERATION = 10.162;
	public static final double DRIVEPATH_MAX_JERK = 100;

	public static double MAX_VEL = 1.5;
	public static double MAX_ACC = 22.25;
	public static double MAX_JRK = 4;
	public static double KP = 4.0;
	public static double KD = 0.0;
	public static double GFACT = 2.0;

	public static boolean calibrate = false;
	public static Integer strategyOption = STRATEGY_SAME_SIDE_SCALE;
	// private static OI oi;

	public static double powerScale = 0.6;

	private CommandGroup autonomousCommand;

	SendableChooser<Integer> positionChooser = new SendableChooser<>();
	SendableChooser<Integer> strategyChooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		driveTrain = new DriveTrain();
		elevator = new Elevator();
		cubeHandler = new CubeHandler();
		cameras = new Cameras();
		DIOs = new DIOSwitches();
		reset();

		// oi = new OI();

		setDashboardData();
	}

	@Override
	public void robotPeriodic() {
		SmartDashboard.putNumber("Elevator", Robot.elevator.getPosition());
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		showSwitchesState();
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		System.out.println("autonomousInit");

		Timer timer = new Timer();
		timer.start();

		String gameMessage = DriverStation.getInstance().getGameSpecificMessage();
		while ((gameMessage.equals("")) && timer.get() < 1) {
			fmsData = DriverStation.getInstance().getGameSpecificMessage();
		}
		System.out.println(fmsData);
		setDashboardFMSString();
		// setDashboardData();
		getRobotPosition();
		getStrategyChoice();
		showSwitchesState();
		// getAutoTargets();

		if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

		SmartDashboard.putBoolean("Error", false);
		if (calibrate) {
            autonomousCommand = new CommandGroup();
            autonomousCommand.addSequential(new Calibrate());
        } else {
            autonomousCommand = new AutoSelector();
        }
		driveTrain.reset();
		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
            autonomousCommand.start();
        }
        System.out.println("Auto command: " + autonomousCommand);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		driveTrain.reset();

		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		showSwitchesState();
		// DIOs.getPreferences();
		// System.out.println(robotPosition);
	}

	void reset() {
		driveTrain.reset();
		elevator.reset();
	}

	private void setDashboardData() {
		positionChooser.addObject("Left", POSITION_LEFT);
		positionChooser.addDefault("Center", POSITION_CENTER);
		positionChooser.addObject("Right", POSITION_RIGHT);

		SmartDashboard.putData("Position", positionChooser);

		strategyChooser = new SendableChooser<>();
		strategyChooser.addObject("Same Side Switch", STRATEGY_SAME_SIDE_SWITCH);
		strategyChooser.addDefault("Same Side Scale", STRATEGY_SAME_SIDE_SCALE);
		strategyChooser.addObject("Other Side Scale", STRATEGY_OPPOSITE_SCALE);
		strategyChooser.addObject("Two Cube Auto", STRATEGY_TWO_CUBES);

		SmartDashboard.putData("Strategy Selector", strategyChooser);

		SmartDashboard.putNumber("MAX_VEL", MAX_VEL);
		SmartDashboard.putNumber("MAX_ACC", MAX_ACC);
		SmartDashboard.putNumber("MAX_JRK", MAX_JRK);
		SmartDashboard.putNumber("KP", KP);

		SmartDashboard.putNumber("GFACT", GFACT);
		SmartDashboard.putBoolean("Use Gyro", true);
		SmartDashboard.putString("Target", "Calculating");

		SmartDashboard.putBoolean("Calibrate", calibrate);
		SmartDashboard.putBoolean("Publish Path", false);
		SmartDashboard.putNumber("Auto Scale", powerScale);

		// SmartDashboard.putBoolean("Grabber Intake", false);
		// SmartDashboard.putBoolean("Grabber Output", false);
		// SmartDashboard.putBoolean("Grabber Arms", false);
	}

	void getDashboardData() {
		useGyro = SmartDashboard.getBoolean("Use Gyro", useGyro);
		MAX_VEL = SmartDashboard.getNumber("MAX_VEL", MAX_VEL);
		MAX_ACC = SmartDashboard.getNumber("MAX_ACC", MAX_ACC);
		MAX_JRK = SmartDashboard.getNumber("MAX_JRK", MAX_JRK);
		GFACT = SmartDashboard.getNumber("GFACT", GFACT);
		KP = SmartDashboard.getNumber("KP", KP);
		powerScale = SmartDashboard.getNumber("Auto Scale", powerScale);
		calibrate = SmartDashboard.getBoolean("Calibrate", calibrate);
	}

	int getDashboardPosition() {
		return positionChooser.getSelected();
	}

	int getDashboardStrategy() {
		return strategyChooser.getSelected();
	}

	void setDashboardFMSString() {
		SmartDashboard.putString("FMS Data", fmsData);
	}

	private void getStrategyChoice() {
		if (!useHardware) {
			strategyOption = getDashboardStrategy();
		} else {
			strategyOption = DIOs.getStrategy();
		}
	}

	private void getRobotPosition() {
		if (!useHardware) {
			robotPosition = getDashboardPosition();
		} else {
			robotPosition = DIOs.getPosition();
		}
	}

	void showSwitchesState() {
		SmartDashboard.putNumber("PositionSwitches", DIOs.getPosition());
		SmartDashboard.putNumber("TargetSwitches", DIOs.getStrategy());

	}
}
