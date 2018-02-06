package org.usfirst.frc.team159.robot.commands;

import java.util.ArrayList;
import java.util.Random;

import org.usfirst.frc.team159.Point;
import org.usfirst.frc.team159.robot.PhysicalConstants;
import org.usfirst.frc.team159.robot.Robot;

//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 *
 */
public class DrivePath extends Command implements PhysicalConstants {
	private static final double TIME_STEP = 0.02;
	// double MAX_VEL = 1.3635; //1.8
	// double MAX_ACC = 18.7; //14
	// double MAX_JRK = 497.9; //116
	private static final double MAX_VEL = 2.88; // 2.75 m/s measured, but reduced to avoid exceeding max on outside wheels when turning
	private static final double MAX_ACC = 26.7;
	private static final double MAX_JRK = 10;
	public static double KP = 4;
	private static final double KI = 0.0;
	private static final double KD = 0.0;
	private static double KV = 1.0 / MAX_VEL;
	private static final double KA = 0.0;
	private static final double GFACT = 2.0;
	private static final double wheelbase = inchesToMeters(26);
	private Trajectory trajectory;
	private Trajectory leftTrajectory;
	private Trajectory rightTrajectory;
	private DistanceFollower leftFollower;
	private DistanceFollower rightFollower;
	//private Trajectory.Config config;
	//TankModifier modifier;
	private Timer timer;
	private static final boolean printCalculatedTrajectory = false;
	private static final boolean printCalculatedPath = false;
	private static final boolean useGyro = false;
	private static final boolean debugCommand = false;
	private static final boolean printPath = true;
	private static final boolean publishPath = true;
	
//	private static int plotId = 0;
	private int pathIndex = 0;

	private static final int LEFT_POSITION = 0;
	private static final int CENTER_POSITION = 1;
	private static final int RIGHT_POSITION = 2;

	private final double runtime;

	private static final Point hookEndPoint = new Point(ROBOT_TO_SWITCH_CENTER, HOOK_Y_DISTANCE);

	private static final Point switchCurveEndPoint = new Point(ROBOT_TO_SWITCH, SWITCH_CENTER_TO_PLATE_EDGE);
	private static final double straightEndPoint = 78;
	private static final double scaleEndPoint = SCALE_HOOK_TURN_Y;

	private static ArrayList<PathData> pathData = new ArrayList<>();
	//static NetworkTableInstance ti=NetworkTableInstance.getDefault();	
	//static NetworkTable table=ti.getTable("datatable");
	//TODO use non deprecated class (edu.wpi.first.networktables.NetworkTable)
	private static NetworkTable table = NetworkTable.getTable("datatable");

	
	DrivePath() {
		requires(Robot.driveTrain);
		
		Sendable position = SmartDashboard.getData("Position");
		SendableChooser<Integer> robotPosition = (SendableChooser<Integer>) position;
		
		String gameMessage = DriverStation.getInstance().getGameSpecificMessage();
		if (gameMessage.equals("")) {
			if (isGameDataEmpty()) {
				gameMessage = generateFMSData();
			} else {
				gameMessage = SmartDashboard.getString("FMS Data", "LLL");
			}
		}
		putStringOnDashboard("FMS Data", gameMessage);

		timer = new Timer();
		timer.start();
		timer.reset();

		//System.out.println(System.getProperty("java.library.path"));
		//System.out.println(wheelbase);

		double maxVelocity = MAX_VEL;
		double maxAcceleration = MAX_ACC;
		double maxJerk = MAX_JRK;
		//double maxVelocity = getNumberOnDashboard("Max Velocity", 1);
		//double maxAcceleration = getNumberOnDashboard("Max Acceleration", 1);
		//double maxJerk = getNumberOnDashboard("Max Jerk", 1);

		KV = 1 / maxVelocity;
		KP = getNumberOnDashboard("P", KP);
		
		trajectory = calculateTrajectory(gameMessage, robotPosition.getSelected(), maxVelocity, maxAcceleration, maxJerk);
		
		// Create the Modifier Object
		TankModifier modifier = new TankModifier(trajectory);
		modifier.modify(wheelbase);
		runtime = trajectory.length() * TIME_STEP;

		// Generate the Left and Right trajectories using the original trajectory as the center

		leftTrajectory = modifier.getLeftTrajectory(); // Get the Left Side
		rightTrajectory = modifier.getRightTrajectory(); // Get the Right Side
		
		//Segment seg = leftTrajectory.segments[0];

		leftFollower = new DistanceFollower(leftTrajectory);
		leftFollower.configurePIDVA(KP, KI, KD, KV, KA);
		rightFollower = new DistanceFollower(rightTrajectory);
		rightFollower.configurePIDVA(KP, KI, KD, KV, KA);
		System.out.format("trajectory length:%d runtime:%f calctime:%f\n", trajectory.length(), runtime, timer.get());

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
		
		printInitializeMessage();
		pathData.clear();
		
		leftFollower.reset();
		rightFollower.reset();
		Robot.driveTrain.reset();
		timer.start();
		timer.reset();
		pathIndex=0;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (trajectory == null) {
			return;
		}
		double leftDistance = feetToMeters(Robot.driveTrain.getLeftDistance());
		double rightDistance = feetToMeters(Robot.driveTrain.getRightDistance());
		
		double rightPower = Robot.scale * leftFollower.calculate(leftDistance);
		double leftPower = Robot.scale * rightFollower.calculate(rightDistance);

		double turn = 0;

		double gyroHeading = Robot.driveTrain.getHeading(); // Assuming the gyro is giving a value in degrees
		double pathfinderHeading = Pathfinder.r2d(leftFollower.getHeading()); // Should also be in degrees

		pathfinderHeading = pathfinderHeading > 180 ? 360 - pathfinderHeading : pathfinderHeading;
		double headingError = pathfinderHeading - gyroHeading;
		if (useGyro)
			turn = GFACT * (-1.0 / 180.0) * headingError;
		if (debugCommand) {
			System.out.format("%f %f %f %f %f %f %f\n", timer.get(), leftDistance, rightDistance, pathfinderHeading, gyroHeading, rightPower + turn, leftPower - turn);
		}

		if (printPath)
			debugPathError();		
		if (publishPath)
			addPlotData();
		
		pathIndex++;

		// this is reversed because we found it to be reversed, don't change unless you know what you're doing
		Robot.driveTrain.tankDrive(leftPower + turn, rightPower - turn);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
        return trajectory == null || timer.get() - runtime > 0.5 || leftFollower.isFinished() && rightFollower.isFinished();

    }

	// Called once after isFinished returns true
	protected void end() {
		printEndMessage();
		if(publishPath)
			publish(pathData,4);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

	
	private Waypoint[] calculateScalePoints(double y) {
		Waypoint[] waypoints = new Waypoint[3];
		waypoints[0] = new Waypoint(0, 0, 0);
		waypoints[1] = new Waypoint(SCALE_HOOK_TURN_X, 0, Pathfinder.d2r(45));
		waypoints[2] = new Waypoint(ROBOT_TO_SCALE_CENTER, y, Pathfinder.d2r(90));
		return waypoints;
	}

	private Waypoint[] calculateHookPoints(double x, double y) {
		Waypoint[] waypoints = new Waypoint[3];
		waypoints[0] = new Waypoint(0, 0, 0);
		waypoints[1] = new Waypoint(x - Math.abs(y), 0, 0);
		if (y < 0) {
			waypoints[2] = new Waypoint(x, y, Pathfinder.d2r(-90));
		} else {
			waypoints[2] = new Waypoint(x, y, Pathfinder.d2r(90));
		}
		return waypoints;
	}

	private Waypoint[] calculateStraightPoints(double x) {
        return new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(x, 0, 0) };
	}

	private Waypoint[] calculateSwitchCurvePoints(double x, double y) {
		Waypoint[] waypoints = new Waypoint[3];
		waypoints[0] = new Waypoint(0, 0, 0);
		if (y < 0) {
			waypoints[1] = new Waypoint(x / 2, y / 2, Pathfinder.d2r(-45));
		} else {
			waypoints[1] = new Waypoint(x / 2, y / 2, Pathfinder.d2r(45));
		}
		waypoints[2] = new Waypoint(x, y, 0);
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
	
	private Trajectory calculateTrajectory(String gameData, int robotPosition, double maxVelocity, double maxAcceleration, double maxJerk) {
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, TIME_STEP, maxVelocity, maxAcceleration, maxJerk);
		
		Waypoint[] waypoints = calculatePathWaypoints(gameData, robotPosition);
		Trajectory calculatedTrajectory = Pathfinder.generate(waypoints, config);
		
		if (calculatedTrajectory == null) {
			System.out.println("Uh-Oh! Trajectory could not be generated!\n");
		}
		return calculatedTrajectory;
	}

	private Waypoint[] calculatePathWaypoints(String gameData, int robotPosition) {
		Waypoint[] returnWaypoints = null;
        switch (robotPosition) {
            case CENTER_POSITION:
                if (gameData.charAt(0) == 'R') {
                    returnWaypoints = calculateSwitchCurvePoints(switchCurveEndPoint.x, switchCurveEndPoint.y);
                } else {
                    returnWaypoints = mirrorWaypoints(
                            calculateSwitchCurvePoints(switchCurveEndPoint.x, switchCurveEndPoint.y));
                }
                break;
            case RIGHT_POSITION:
                if (!isStraightPathForced()) {
                    if (gameData.charAt(1) == 'R' && gameData.charAt(0) == 'R') {
                        if (isScalePreferredOverSwitch()) {
                            returnWaypoints = calculateScalePoints(scaleEndPoint);
                        } else {
                            returnWaypoints = calculateHookPoints(hookEndPoint.x, hookEndPoint.y);
                        }
                    } else if (gameData.charAt(0) == 'R') {
                        returnWaypoints = calculateHookPoints(hookEndPoint.x, hookEndPoint.y);
                    } else {
                        returnWaypoints = calculateStraightPoints(straightEndPoint);
                    }
                } else {
                    returnWaypoints = calculateStraightPoints(straightEndPoint);
                }
                break;
            case LEFT_POSITION:
                if (!isStraightPathForced()) {
                    if (gameData.charAt(1) == 'L' && gameData.charAt(0) == 'L') {
                        if (isScalePreferredOverSwitch()) {
                            returnWaypoints = mirrorWaypoints(calculateScalePoints(scaleEndPoint));
                        } else {
                            returnWaypoints = mirrorWaypoints(calculateHookPoints(hookEndPoint.x, hookEndPoint.y));
                        }
                    } else if (gameData.charAt(0) == 'L') {
                        returnWaypoints = mirrorWaypoints(calculateHookPoints(hookEndPoint.x, hookEndPoint.y));
                    } else {
                        returnWaypoints = calculateStraightPoints(straightEndPoint);
                    }
                } else {
                    returnWaypoints = calculateStraightPoints(straightEndPoint);
                }
                break;
        }
        assert returnWaypoints != null;
        return waypointsInchesToMeters(returnWaypoints);
	}

	private String generateFMSData() {
		Random random = new Random();
		StringBuilder data = new StringBuilder();
		while (data.length() < 3) {
			if (random.nextInt(2) == 0) {
				data.append("R");
			} else {
				data.append("L");
			}
		}
		return data.toString();
	}

	private static double inchesToMeters(double inches) {
		return inches * 0.0254;
	}

	private static double metersToInches(double meters) {
		return meters / 0.0254;
	}

	private void debugPathError() {
		double leftDistance = 12 * (Robot.driveTrain.getLeftDistance());
		double rightDistance = 12 * (Robot.driveTrain.getRightDistance());
		Segment segment = leftTrajectory.segments[pathIndex];
		double leftTarget = metersToInches(segment.position);
		segment = rightTrajectory.segments[pathIndex];
		double rightTarget = metersToInches(segment.position);
		
		//double headingTarget = Pathfinder.r2d(segment.heading);
		//double currentHeading = Robot.driveTrain.getHeading();

		System.out.format("%f %f %f %f %f\n", timer.get(), leftDistance, leftTarget, rightDistance, rightTarget);

	}


	private static double feetToMeters(double feet) {
		return 2.54 * 12 * feet / 100;
	}

	private void putStringOnDashboard(String name, String data) {
		SmartDashboard.putString(name, data);
	}
	
	private double getNumberOnDashboard(String name, double defaultValue) {
		return SmartDashboard.getNumber(name, defaultValue);
	}
	
	private void printInitializeMessage() {
		System.out.println("DrivePath.initialize()");
	}
	
	private void printEndMessage() {
		System.out.println("DrivePath.end()");
	}
	
	private boolean isGameDataEmpty() {
		return SmartDashboard.getString("FMS Data", "").equals("") || SmartDashboard.getString("FMS Data", "???").equals("???");
	}

	private boolean isScalePreferredOverSwitch() {
		return SmartDashboard.getBoolean("Prefer Scale", true);
	}

	private boolean isStraightPathForced() {
		return SmartDashboard.getBoolean("Force Straight Path", false);
	}
	
	private void addPlotData() {
		PathData pd = new PathData();
		
		pd.tm = timer.get();
		pd.d[0] = 12 * (Robot.driveTrain.getLeftDistance());
		pd.d[2] = 12 * (Robot.driveTrain.getRightDistance());
		Segment ls = leftTrajectory.get(pathIndex);
		Segment rs = rightTrajectory.get(pathIndex);
		pd.d[1] = metersToInches (ls.position);
		pd.d[3] = metersToInches (rs.position);
		pd.d[4] = Robot.driveTrain.getHeading(); // Assuming the gyro is giving a value in degrees
		double th = Pathfinder.r2d(rs.heading); // Should also be in degrees
		pd.d[5] = th > 180 ? th - 360 : th; // convert to signed angle fixes problem:th 0->360 gh:-180->180
		pathData.add(pd);
	}

	private static void publish(ArrayList<PathData> d, int traces) {
    	double info[] = new double[3];
    	int points=d.size();
    	info[0]=0;
    	info[1]=traces;
    	info[2]=points;
    
    	System.out.println("Publishing Plot Data");
    	//table.putValue("NewPlot", NetworkTableValue.makeDoubleArray(info));
		table.putNumberArray("NewPlot", info);

    	for (int i = 0; i < points; i++) {
    		PathData pd=d.get(i);
			double data[] = new double[traces+2];
			data[0]=(double)i;
			data[1]=pd.tm;
			for (int j = 0; j < traces; j++) {
				data[j+2]=pd.d[j];
			}
			table.putNumberArray("PlotData"+i, data);
		}
    }
	public class PathData {
		static final int DATA_SIZE=6;
		double tm=0;
		double d[]=new double[DATA_SIZE];
	}

}
