package org.usfirst.frc.team159.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team159.robot.commands.Autonomous;
import org.usfirst.frc.team159.robot.commands.Calibrate;
import org.usfirst.frc.team159.robot.commands.DropGrabber;
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
public class Robot extends IterativeRobot implements RobotMap {

    public static Elevator elevator;
    public static CubeHandler cubeHandler;
    public static DriveTrain driveTrain;
    private static Cameras cameras;
    public static DIOSwitches DIOs;

    public static boolean useGyro = false;
    public static boolean preferScale = false;
    public static final boolean useHardware = true;

    public static int robotPosition = -1;
    public static String fmsData = "LLL";
    public static double MAX_VEL = 1.5;
    public static double MAX_ACC = 22.25;
    public static double MAX_JRK = 4;
    public static double KP = 4.0;
    public static double KD = 0.0;
    public static double GFACT = 2.0;

    public static boolean calibrate = false;
    // public static Integer strategyOption = STRATEGY_SAME_SIDE_SCALE;

    public static double scale = 0.6;

    private CommandGroup autonomousCommand;

    SendableChooser<Integer> positionChooser = new SendableChooser<>();

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

        putValuesOnSmartDashboard();
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
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        getDashboardData();
        robotPosition = getPosition();
        getFMSData();

        if (SmartDashboard.getBoolean("Calibrate", false)) {
            autonomousCommand = new CommandGroup();
            autonomousCommand.addSequential(new Calibrate());
        } else {
            autonomousCommand = new Autonomous();
        }

        driveTrain.reset();

        if (autonomousCommand != null) {
            autonomousCommand.start();
        }
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
    }

    private void putValuesOnSmartDashboard() {
        positionChooser.addObject("Left", 0);
        positionChooser.addDefault("Center", 1);
        positionChooser.addObject("Right", 2);
        SmartDashboard.putData("Position", positionChooser);
        SmartDashboard.putNumber("Max Velocity", MAX_VEL);
        SmartDashboard.putNumber("Max Acceleration", MAX_ACC);
        SmartDashboard.putNumber("Max Jerk", MAX_JRK);
        SmartDashboard.putNumber("KP", KP);
        SmartDashboard.putNumber("GFACT", GFACT);
        SmartDashboard.putBoolean("Use Gyro", useGyro);
        SmartDashboard.putString("Target", "Calculating");
        SmartDashboard.putString("FMS Data", "RLL");
        SmartDashboard.putBoolean("Calibrate", false);
        SmartDashboard.putBoolean("Publish Path", false);
        SmartDashboard.putBoolean("Prefer Scale", preferScale);
        SmartDashboard.putBoolean("Grabber Intake", false);
        SmartDashboard.putBoolean("Grabber Output", false);
        SmartDashboard.putBoolean("Grabber Arms", false);
    }

    void getDashboardData() {
        useGyro = SmartDashboard.getBoolean("Use Gyro", useGyro);
        MAX_VEL = SmartDashboard.getNumber("MAX_VEL", MAX_VEL);
        MAX_ACC = SmartDashboard.getNumber("MAX_ACC", MAX_ACC);
        MAX_JRK = SmartDashboard.getNumber("MAX_JRK", MAX_JRK);
        GFACT = SmartDashboard.getNumber("GFACT", GFACT);
        KP = SmartDashboard.getNumber("KP", KP);
        scale = SmartDashboard.getNumber("Auto Scale", scale);
        calibrate = SmartDashboard.getBoolean("Calibrate", calibrate);
        preferScale = SmartDashboard.getBoolean("Prefer Scale", preferScale);
    }

    private int getPosition() {
        if (!useHardware) {
            return positionChooser.getSelected();
        } else {
            return DIOs.getPosition();
        }
    }

    void getFMSData() {
        fmsData = DriverStation.getInstance().getGameSpecificMessage();
        if ((fmsData.equals("") || fmsData == null)) {
            System.out.println("WARNING getGameSpecificMessage() not valid at autonomousInit!!");
            Timer timer = new Timer();
            timer.start();
            timer.reset();
            double tm = 0;
            while ((fmsData.equals("") || fmsData == null) && (tm = timer.get() < 2)) {
                fmsData = DriverStation.getInstance().getGameSpecificMessage();
                if (tm >= 2)
                    System.out.println("WARNING getFMSMessage timed out !!");
            }
        }
        System.out.println("Robot.getFMSData FMS=" + fmsData);
        putFMSDataOnDashboard(fmsData);
    }

    private void putFMSDataOnDashboard(String data) {
        SmartDashboard.putString("FMS Data", data);
    }

}
