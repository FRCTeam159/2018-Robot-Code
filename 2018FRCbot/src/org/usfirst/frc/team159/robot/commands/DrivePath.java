package org.usfirst.frc.team159.robot.commands;

import java.util.ArrayList;

import org.usfirst.frc.team159.robot.Constants;
import org.usfirst.frc.team159.robot.PhysicalConstants;
import org.usfirst.frc.team159.robot.RobotMap;

import org.usfirst.frc.team159.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class DrivePath extends Command implements PhysicalConstants, RobotMap, Constants {

    // double MAX_VEL = 1.3635; //1.8
    // double MAX_ACC = 18.7; //14
    // double MAX_JRK = 497.9; //116
    private static final double TIME_STEP = 0.02;

    private static final double wheelbase = inchesToMeters(26);
    private static final boolean printCalculatedTrajectory = false;
    private static final boolean printCalculatedPath = false;
    private static final boolean debugCommand = false;
    private static final boolean printPath = false;
    static int plotCount = 0;
    private static boolean publishPath = false;
    //static NetworkTableInstance ti=NetworkTableInstance.getDefault();
    //static NetworkTable table=ti.getTable("datatable");
    //TODO use non deprecated class (edu.wpi.first.networktables.NetworkTable)
    private static NetworkTable table = NetworkTable.getTable("datatable");
//    final double i2m = 0.0254;
    final double m2i = (1.0 / 0.0254);
    private double runTime = 0;
    private double lastHeading = 0;
    private int pathPoints = 0;
    private int target = 0;
    private boolean mirror;
    boolean reverse;
    private Trajectory trajectory;
    private Trajectory leftTrajectory;
    private Trajectory rightTrajectory;
    private DistanceFollower leftFollower;
    private DistanceFollower rightFollower;
    private int pathIndex = 0;
    private Timer timer = new Timer();
    private ArrayList<PathData> pathDataList = new ArrayList<>();

    public DrivePath(int target, boolean mirror, boolean reverse) {
        requires(Robot.driveTrain);
        requires(Robot.elevator);

        this.target = target;
        this.mirror = mirror;
        this.reverse = reverse;

        publishPath = SmartDashboard.getBoolean("Publish Path", false);

        System.out.println("Position=" + Robot.robotPosition);

        double MAX_VEL = Robot.MAX_VEL;
        double MAX_ACC = Robot.MAX_ACC;
        double MAX_JRK = Robot.MAX_JRK;
        double KP = Robot.KP;
        double KI = 0.0;
        double KD = Robot.KP;
        double KV = 1.0 / MAX_VEL;
        double KA = 0.0;

        //System.out.println("FMS=" + gameMessage);

        //putFMSDataOnDashboard(gameMessage);
        timer.start();
        timer.reset();

        Waypoint[] waypoints = calculatePath(target);
        for (Waypoint waypoint : waypoints) {
            System.out.println(m2i * waypoint.x + " " + m2i * waypoint.y + " " + waypoint.angle);
        }

        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, TIME_STEP,
                MAX_VEL, MAX_ACC, MAX_JRK);

        trajectory = Pathfinder.generate(waypoints, config);

        if (trajectory == null) {
            SmartDashboard.putString("Target", "ERROR");
            runTime = 0;
            return;
        }

        // Create the Modifier Object
        TankModifier modifier = new TankModifier(trajectory);
        modifier.modify(wheelbase);
        runTime = trajectory.length() * TIME_STEP;

        // Generate the Left and Right trajectories using the original trajectory as the center

        leftTrajectory = modifier.getLeftTrajectory(); // Get the Left Side
        rightTrajectory = modifier.getRightTrajectory(); // Get the Right Side

        leftFollower = new DistanceFollower(leftTrajectory);
        leftFollower.configurePIDVA(KP, KI, KD, KV, KA);
        rightFollower = new DistanceFollower(rightTrajectory);
        rightFollower.configurePIDVA(KP, KI, KD, KV, KA);
        System.out.format("trajectory length:%data runtime:%f calctime:%f\n", trajectory.length(), runTime, timer.get());

        if (printCalculatedTrajectory) {
            double time = 0;
            for (int i = 0; i < trajectory.length(); i++) {
                Segment segment = trajectory.get(i);
                System.out.format("%f %f %f %f %f %f \n", time, segment.x, segment.y, segment.velocity, segment.acceleration, segment.heading);
                time += segment.dt;
            }
        }

        if (printCalculatedPath) {
            double time = 0;
            for (int i = 0; i < trajectory.length(); i++) {
                Segment centerSegment = trajectory.get(i);
                Segment leftSegment = leftTrajectory.get(i);
                Segment rightSegment = rightTrajectory.get(i);
                System.out.format("%f %f %f %f %f\n", time, leftSegment.x, leftSegment.y, rightSegment.x, rightSegment.y);
                time += centerSegment.dt;
            }
        }
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        if (trajectory == null) {
            return;
        }
        System.out.println("DrivePath.initialize:" + target);

        pathDataList.clear();
        if (!publishPath) {
            plotCount = 0;
        }

        leftFollower.reset();
        rightFollower.reset();
        Robot.driveTrain.reset();
        timer.start();
        timer.reset();
        pathIndex = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (trajectory == null) {
            return;
        }
        double scale = Robot.powerScale;

        double leftDistance = Robot.driveTrain.getLeftDistance();
        double rightDistance = Robot.driveTrain.getRightDistance();
        if (reverse) {
            leftDistance = -leftDistance;
            rightDistance = -rightDistance;
        }
        double left = leftFollower.calculate(feet2meters(leftDistance)); // reversal ?
        double right = rightFollower.calculate(feet2meters(rightDistance));
        double turn = 0;
        double gyroHeading = Robot.driveTrain.getHeading(); // Assuming the gyro is giving a value in degrees
        if (reverse) {
            gyroHeading = -gyroHeading;
        }
        gyroHeading = unwrap(lastHeading, gyroHeading);

        double followerHeading = Pathfinder.r2d(leftFollower.getHeading()); // Should also be in degrees
        followerHeading = followerHeading > 180 ? followerHeading - 360 : followerHeading; // convert to signed angle fixes problem:th 0->360 gh:-180->180
        double headingError = followerHeading - gyroHeading;
        if (Robot.useGyro)
            turn = Robot.GFACT * (-1.0 / 180.0) * headingError;
        double leftValue = left + turn;
        double rightValue = right - turn;
        leftValue *= scale;
        rightValue *= scale;
        if (Math.abs(leftValue) > 1.0 || Math.abs(rightValue) > 1.0)
            SmartDashboard.putBoolean("Error", true);
        double currentTime = timer.get();

        if (debugCommand) {
            System.out.format("%f %f %f %f %f %f %f\n", currentTime, leftDistance, rightDistance, gyroHeading, followerHeading, leftValue, rightValue);
        }
        if (publishPath) {
            addPlotData(leftDistance, rightDistance, gyroHeading);
        }
        if (reverse) {
            Robot.driveTrain.tankDrive(-leftValue, -rightValue);
        } else {
            Robot.driveTrain.tankDrive(leftValue, rightValue);
        }
        Robot.cubeHandler.hold();
        pathIndex++;
        lastHeading = gyroHeading;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (trajectory == null) {
            return true;
        }
        if ((pathIndex >= pathPoints) || (leftFollower.isFinished() && rightFollower.isFinished())) {
            return true;
        } else if ((timer.get() - runTime) > 1) {
            System.out.println("DrivePath Timeout Expired");
            return true;
        }
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        System.out.println("DrivePath.end()");
        if (publishPath) {
            publish(pathDataList, 6);
        }
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }

    private static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    private static double metersToInches(double meters) {
        return meters / 0.0254;
    }

    private static double feetToMeters(double feet) {
        return 2.54 * 12 * feet / 100;
    }

    private static void publish(ArrayList<PathData> dataList, int traces) {
        double info[] = new double[3];
        int points = dataList.size();
        info[0] = plotCount;
        info[1] = traces;
        info[2] = points;

        System.out.println("Publishing Plot Data");
        // table.putValue("NewPlot", NetworkTableValue.makeDoubleArray(info));
        table.putNumberArray("NewPlot" + plotCount, info);

        for (int i = 0; i < points; i++) {
            PathData pathData = dataList.get(i);
            double data[] = new double[traces + 2];
            data[0] = (double) i;
            data[1] = pathData.time;
            for (int j = 0; j < traces; j++) {
                data[j + 2] = pathData.data[j];
            }
            table.putNumberArray("PlotData" + i, data);
        }
        dataList.clear();
        plotCount++;
    }

    double feet2meters(double x) {
        return 12 * x * 0.0254;
    }

    double unwrap(double previousAngle, double newAngle) {
        double degrees = newAngle - previousAngle;
        degrees = degrees >= 180 ? degrees - 360 : (degrees <= -180 ? degrees + 360 : degrees);
        return previousAngle + degrees;
    }

    private Waypoint[] calculateStraightPoints() {
        double x = ROBOT_TO_SWITCH + 12;
        Waypoint[] waypoints = new Waypoint[2];
        waypoints[0] = new Waypoint(0, 0, 0);
        waypoints[1] = new Waypoint(x, 0, 0);
        return waypoints;
    }

    private Waypoint[] calculateSecondCenterSwitchPoints() {
        double x = 70;// back up distance can be shorter than start distance
        double y = SWITCH_CENTER_TO_PLATE_EDGE;
        boolean delta = mirror ^ reverse; // true if the target is right side

        y -= delta ? ROBOT_Y_OFFSET_FROM_CENTER - 6 : 0;

        Waypoint[] waypoints = new Waypoint[3];
        waypoints[0] = new Waypoint(0, 0, 0);
        waypoints[1] = new Waypoint(x / 2, y / 2, Pathfinder.d2r(45));
        waypoints[2] = new Waypoint(x, y, 0);
        if (mirror)
            return mirrorWaypoints(waypoints);
        else
            return waypoints;
    }

    private Waypoint[] calculateCenterSwitchPoints() {
        double x = ROBOT_TO_SWITCH;
        double y = SWITCH_CENTER_TO_PLATE_EDGE;
        y -= mirror ? ROBOT_Y_OFFSET_FROM_CENTER : 0;

        Waypoint[] waypoints = new Waypoint[3];
        waypoints[0] = new Waypoint(0, 0, 0);
        waypoints[1] = new Waypoint(x / 2, y / 2, Pathfinder.d2r(45));
        waypoints[2] = new Waypoint(x + 4, y, 0);
        if (mirror)
            return mirrorWaypoints(waypoints);
        else
            return waypoints;
    }

    /**
     * Same Side Switch Path
     * - Fires cube from Switch plate corner (45 degrees)
     */
    private Waypoint[] calculateSideSwitchPoints() {
        double y = SWITCH_HOOK_Y_DISTANCE - 12;
        double x = ROBOT_TO_SWITCH;
        Waypoint[] waypoints = new Waypoint[3];
        waypoints[0] = new Waypoint(0, 0, 0);
        waypoints[1] = new Waypoint(x - 2 * y, 0, 0);
        waypoints[2] = new Waypoint(x, y, Pathfinder.d2r(45));
        if (mirror)
            return mirrorWaypoints(waypoints);
        else
            return waypoints;
    }

    /**
     * Same Side Switch Path
     * - Fires cube from Switch plate outside edge (90 degrees)
     */
    private Waypoint[] calculateSideSwitchHookPoints() {
        double y = SWITCH_HOOK_Y_DISTANCE;
        double x = ROBOT_TO_SWITCH_CENTER + 12;
        Waypoint[] waypoints = new Waypoint[3];
        waypoints[0] = new Waypoint(0, 0, 0);
        waypoints[1] = new Waypoint(x - 2.25 * y, 0, 0);
        waypoints[2] = new Waypoint(x, y, Pathfinder.d2r(90));
        if (mirror) {
            return mirrorWaypoints(waypoints);
        } else {
            return waypoints;
        }
    }

    /**
     * Same Side Scale Path
     * - Fires cube from Scale plate corner (45 degrees)
     */
    private Waypoint[] calculateSideScalePoints() {
        Waypoint[] waypoints = new Waypoint[4];
        double y = SCALE_HOOK_Y_DISTANCE;
        double x = ROBOT_TO_SCALE_X - 6;
        waypoints[0] = new Waypoint(0, 0, 0);

        waypoints[1] = new Waypoint(x - 6 * y, -6, 0);
        waypoints[2] = new Waypoint(x - 2 * y, -6, 0);
        waypoints[3] = new Waypoint(x, y - 6, Pathfinder.d2r(45));
        if (mirror)
            return mirrorWaypoints(waypoints);
        else
            return waypoints;
    }

    /**
     * Same Side Scale Path
     * - Fires cube from Scale plate outside edge (90 degrees)
     */
    private Waypoint[] calculateSideScaleHookPoints() {
        double y = SCALE_HOOK_Y_DISTANCE;
        double x = ROBOT_TO_SCALE_CENTER;
        Waypoint[] waypoints = new Waypoint[4];
        waypoints[0] = new Waypoint(0, 0, 0);
        waypoints[1] = new Waypoint(x - 6 * y, -12, 0);
        waypoints[2] = new Waypoint(x - 2 * y, -12, 0);
        waypoints[3] = new Waypoint(x, y - 9, Pathfinder.d2r(90));
        if (mirror)
            return mirrorWaypoints(waypoints);
        else
            return waypoints;
    }

    // calculate second switch path in 2 cube auto
    private Waypoint[] calculateSecondSwitchPoints() {
        Waypoint[] waypoints = new Waypoint[3];
        waypoints[0] = new Waypoint(0, 0, Pathfinder.d2r(0));
        waypoints[1] = new Waypoint(40, 0, Pathfinder.d2r(0));
        waypoints[2] = new Waypoint(70, 20, Pathfinder.d2r(35)); // best values by trial and error
        if (mirror)
            return mirrorWaypoints(waypoints);
        else
            return waypoints;
    }

    private Waypoint[] calculateOtherScalePoints() {
        Waypoint[] waypoints = new Waypoint[5];
        waypoints[0] = new Waypoint(0, 0, 0);
        waypoints[1] = new Waypoint(130, 0, 0);
        waypoints[2] = new Waypoint(220, 90, Pathfinder.d2r(90));
        waypoints[3] = new Waypoint(220, 135, Pathfinder.d2r(90));
        waypoints[4] = new Waypoint(270, 175, Pathfinder.d2r(-15));
        if (mirror)
            return mirrorWaypoints(waypoints);
        else
            return waypoints;
    }

    private Waypoint[] calculateOtherSwitchPoints() {
        Waypoint[] waypoints = new Waypoint[5];
        waypoints[0] = new Waypoint(0, 0, 0);
        waypoints[1] = new Waypoint(150, 0, 0);
        waypoints[2] = new Waypoint(230, 50, Pathfinder.d2r(90));
        waypoints[3] = new Waypoint(230, 150, Pathfinder.d2r(90));
        waypoints[4] = new Waypoint(200, 180, -Pathfinder.d2r(180));
        if (mirror)
            return mirrorWaypoints(waypoints);
        else
            return waypoints;
    }

    private Waypoint[] mirrorWaypoints(Waypoint[] waypoints) {
        Waypoint[] newWaypoints = new Waypoint[waypoints.length];
        for (int i = 0; i < waypoints.length; i++) {
            newWaypoints[i] = new Waypoint(waypoints[i].x, -waypoints[i].y, -waypoints[i].angle);
        }
        return newWaypoints;
    }

    private Waypoint[] waypointsInchesToMeters(Waypoint[] waypoints) {
        Waypoint[] newWaypoints = new Waypoint[waypoints.length];
        for (int i = 0; i < waypoints.length; i++) {
            newWaypoints[i] = new Waypoint(inchesToMeters(waypoints[i].x), inchesToMeters(waypoints[i].y),
                    waypoints[i].angle);
        }
        return newWaypoints;
    }

    private Waypoint[] calculatePath(int target) {
        Waypoint[] returnWaypoints = null;
        switch (target) {
            case TARGET_GO_STRAIGHT:
                returnWaypoints = calculateStraightPoints();
                break;
            case TARGET_CENTER_SWITCH:
                returnWaypoints = calculateCenterSwitchPoints();
                break;
            case TARGET_SAME_SWITCH:
                returnWaypoints = calculateSideSwitchPoints();
                break;
            case TARGET_SAME_SCALE:
                returnWaypoints = calculateSideScalePoints();
                break;
            case TARGET_OTHER_SCALE:
                returnWaypoints = calculateOtherScalePoints();
                break;
            case TARGET_SIDE_TWO_CUBES:
                returnWaypoints = calculateSecondSwitchPoints();
                break;
            case TARGET_CENTER_TWO_CUBES:
                returnWaypoints = calculateSecondCenterSwitchPoints();
                break;
        }
        return waypointsInchesToMeters(returnWaypoints);
    }

    private void debugPathError(double ld, double rd, double g) {
        double leftDistance = 12 * ld;
        double rightDistance = 12 * rd;
        Segment segment = leftTrajectory.segments[pathIndex];
        double leftTarget = metersToInches(segment.position);
        segment = rightTrajectory.segments[pathIndex];
        double rightTarget = metersToInches(segment.position);
        System.out.format("%f %f %f %f %f\n", timer.get(), leftDistance, leftTarget, rightDistance, rightTarget);
    }

    private void addPlotData(double ld, double rd, double g) {
        PathData pathData = new PathData();

        pathData.time = timer.get();
        pathData.data[0] = 12 * ld;
        pathData.data[2] = 12 * rd;
        Segment leftSegment = leftTrajectory.get(pathIndex);
        Segment rightSegment = rightTrajectory.get(pathIndex);
        pathData.data[1] = metersToInches(leftSegment.position);
        pathData.data[3] = metersToInches(rightSegment.position);
        pathData.data[4] = g; // Assuming the gyro is giving a value in degrees
        double th = Pathfinder.r2d(rightSegment.heading); // Should also be in degrees
        pathData.data[5] = th > 180 ? th - 360 : th; // convert to signed angle fixes problem:th 0->360 gh:-180->180
        pathDataList.add(pathData);
    }

    public class PathData {
        static final int DATA_SIZE = 6;
        double time = 0;
        double data[] = new double[DATA_SIZE];
    }
}
