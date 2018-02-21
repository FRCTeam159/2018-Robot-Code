package org.usfirst.frc.team159.robot.commands;

import java.util.ArrayList;
import java.util.Random;

import org.usfirst.frc.team159.robot.PhysicalConstants;
import org.usfirst.frc.team159.robot.Robot;
import org.usfirst.frc.team159.robot.subsystems.Elevator;

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
	private static  double GFACT = 2.0;
	private static final double wheelbase = inchesToMeters(26);
	private Trajectory trajectory;
	private Trajectory leftTrajectory;
	private Trajectory rightTrajectory;
	private DistanceFollower leftFollower;
	private DistanceFollower rightFollower;
	//private Trajectory.Config config;
	//TankModifier modifier;
	private static double switchHeight = 30;
	private static double scaleHeight = 55;
	private static final boolean printCalculatedTrajectory = false;
	private static final boolean printCalculatedPath = false;
	private static boolean useGyro = false;
	private static final boolean debugCommand = false;
	private static final boolean printPath = false;

	//	private static int plotId = 0;
	private int pathIndex = 0;

	private static final int LEFT_POSITION = 0;
	private static final int CENTER_POSITION = 1;
	private static final int RIGHT_POSITION = 2;

	private static final int SWITCH = 0;
	private static final int SCALE = 1;
	private static final int NONE = 2;

	private static final int LEFT = 0;
	private static final int RIGHT = 1;

	private int targetObject = SCALE;
	private int targetSide = LEFT;
	
	private String whichObject[]= {"Switch","Scale","Straight"};
	private String whichSide[]= {"Left","Right"};

	private Timer timer = new Timer();
	private Timer pushTimer = new Timer();
	
	private boolean pushing = false;
	
	private final double runtime;
	private double pushTime = 1;
	private double endTime = 0;

//	private static final Point hookEndPoint = new Point(ROBOT_TO_SWITCH_CENTER, HOOK_Y_DISTANCE);

//	private static final Point switchCurveEndPoint = new Point(ROBOT_TO_SWITCH, SWITCH_CENTER_TO_PLATE_EDGE);
	private static final double straightEndPoint = 78;
//	private static final double scaleEndPoint = SCALE_HOOK_TURN_Y;
	
	private static int plotCount = 0;

	private ArrayList<PathData> pathDataList = new ArrayList<>();
	//static NetworkTableInstance ti=NetworkTableInstance.getDefault();	
	//static NetworkTable table=ti.getTable("datatable");
	//TODO use non deprecated class (edu.wpi.first.networktables.NetworkTable)
	private static NetworkTable table = NetworkTable.getTable("datatable");

	private String targetString = whichSide[targetSide] + " " + whichObject[targetObject];

	DrivePath() {
		requires(Robot.driveTrain);
		requires(Robot.elevator);
		
//		Sendable position = SmartDashboard.getData("Position");
//		SendableChooser<Integer> robotPosition = (SendableChooser<Integer>) position;
		
		int robotPosition = -1;
		
		switch (SmartDashboard.getString("Position", "Center").toLowerCase()) {
		
			case "left":
				robotPosition = LEFT_POSITION;
				break;
			case "center":
				robotPosition = CENTER_POSITION;
				break;
			case "right":
				robotPosition = RIGHT_POSITION;
				break;
		}
		
		String gameMessage = DriverStation.getInstance().getGameSpecificMessage();
		if (gameMessage.equals("")) {
			if (isGameDataEmpty()) {
				gameMessage = generateFMSData();
			} else {
				gameMessage = SmartDashboard.getString("FMS Data", "LLL");
			}
		}
		putFMSDataOnDashboard(gameMessage);
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
		GFACT =getNumberOnDashboard("GFACT", GFACT);
		useGyro = SmartDashboard.getBoolean("Use Gyro", false);
		
		trajectory = calculateTrajectory(gameMessage, robotPosition, maxVelocity, maxAcceleration, maxJerk);
		
		targetString = whichSide[targetSide] + " " + whichObject[targetObject];
		SmartDashboard.putString("Target", targetString);
		
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
		System.out.format("trajectory length:%data runtime:%f calctime:%f\n", trajectory.length(), runtime, timer.get());

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
		pathDataList.clear();
		if(!publishPathAllowed()) {
			plotCount = 0;
		}
		
//		Robot.elevator.setElevatorTarget(6);
		
//		if(targetObject == SWITCH) {
		Robot.elevator.setElevatorTarget(Elevator.SWITCH_HEIGHT);
//		} else if(targetObject == SCALE) {
//			Robot.elevator.setElevatorTarget(Elevator.SCALE_HEIGHT);
//		}
		
		leftFollower.reset();
		rightFollower.reset();
		Robot.driveTrain.reset();
		Robot.elevator.enable();
		Robot.cubeHandler.hold();
		timer.start();
		timer.reset();
		pathIndex=0;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (trajectory == null || pushing) {
			return;
		}
		double leftDistance = feetToMeters(Robot.driveTrain.getLeftDistance());
		double rightDistance = feetToMeters(Robot.driveTrain.getRightDistance());
		
		double rightPower =  leftFollower.calculate(leftDistance);
		double leftPower = rightFollower.calculate(rightDistance);

		double turn = 0;

		double gyroHeading = Robot.driveTrain.getHeading(); // Assuming the gyro is giving a value in degrees
		double pathfinderHeading = Pathfinder.r2d(leftFollower.getHeading()); // Should also be in degrees

		pathfinderHeading = pathfinderHeading > 180 ? pathfinderHeading-360 : pathfinderHeading;
		double headingError = pathfinderHeading - gyroHeading;
		if (useGyro) {
			turn = GFACT * (-1.0 / 180.0) * headingError;
		}
		
		double lval = leftPower - turn;
		double rval = rightPower + turn;
		
		lval *= Robot.scale;
		rval *= Robot.scale;

		if (debugCommand) {
			System.out.format("%f %f %f %f %f %f %f\n", timer.get(), leftDistance, rightDistance, pathfinderHeading, gyroHeading, rval, lval);
		}

		if (printPath) {
			debugPathError();
		}
		if (publishPathAllowed()) {
			addPlotData();
		}
		
		pathIndex++;

		// this is reversed because we found it to be reversed, don't change unless you know what you're doing
		Robot.driveTrain.tankDrive(lval, rval);
		//Robot.driveTrain.tankDrive(leftPower - turn, rightPower + turn);

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if(trajectory == null) {
			return true;
		}
		if(pushing) {
			Robot.cubeHandler.output();
			if(pushTimer.get() > pushTime) {
				Robot.cubeHandler.hold();
				return true;
			}
		} else if(leftFollower.isFinished() && rightFollower.isFinished()) {
			if(targetObject == SCALE) {
				Robot.elevator.setElevatorTarget(Elevator.SCALE_HEIGHT);
				if(Robot.elevator.getPosition() > Elevator.SCALE_HEIGHT - Elevator.MOVE_RATE && Robot.elevator.getPosition() < Elevator.SCALE_HEIGHT + Elevator.MOVE_RATE) {
					pushing = true;
					pushTimer.reset();
					pushTimer.start();
				}
			}
			if(targetObject == SWITCH) {
				pushing = true;
				pushTimer.reset();
				pushTimer.start();
			}
		} else if (pushTimer.get() - runtime > 2) {
			System.out.println("DrivePath Timeout Expired");
			return true;
		}		
        return false;
    }

	// Called once after isFinished returns true
	protected void end() {
		printEndMessage();
		if(publishPathAllowed()) {
            publish(pathDataList, 6);
        }
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

	private boolean publishPathAllowed(){
	    return SmartDashboard.getBoolean("Publish Path", false);
    }

	/*private Waypoint[] calculateScalePoints(double y) {
		Waypoint[] waypoints = new Waypoint[3];
		waypoints[0] = new Waypoint(0, 0, 0);
		waypoints[1] = new Waypoint(SCALE_HOOK_TURN_X, 0, Pathfinder.d2r(45));
		waypoints[2] = new Waypoint(ROBOT_TO_SCALE_CENTER, y, Pathfinder.d2r(90));
		return waypoints;
	}*/

	private Waypoint[] calculateCenterSwitchPoints(int targetSide) {
		double x=ROBOT_TO_SWITCH;
		double y=SWITCH_CENTER_TO_PLATE_EDGE;
		//y-=targetSide == RIGHT?ROBOT_Y_OFFSET_FROM_CENTER:0;
		
		Waypoint[] waypoints = new Waypoint[3];
		waypoints[0] = new Waypoint(0, 0, 0);
		waypoints[1] = new Waypoint(x / 2, y / 2, Pathfinder.d2r(45));
		waypoints[2] = new Waypoint(x, y, 0);
		if(targetSide == LEFT)
			return mirrorWaypoints(waypoints);
		else
			return waypoints;
	}

	private Waypoint[] calculateSideSwitchHookPoints(int position) {
		double y=SWITCH_HOOK_Y_DISTANCE;
		double x=ROBOT_TO_SWITCH_CENTER+12;
		Waypoint[] waypoints = new Waypoint[3];
		waypoints[0] = new Waypoint(0, 0, 0);
		waypoints[1] = new Waypoint(x-2.25*y, 0,0);
		waypoints[2] = new Waypoint(x, y, Pathfinder.d2r(90));
		if(position == RIGHT_POSITION) {
			return mirrorWaypoints(waypoints);
		} else {
			return waypoints;
		}
	}
	private Waypoint[] calculateSideScaleHookPoints(int position) {
		double y=SCALE_HOOK_Y_DISTANCE;
		double x=ROBOT_TO_SCALE_CENTER;
		Waypoint[] waypoints = new Waypoint[4];
		waypoints[0] = new Waypoint(0, 0, 0);
		waypoints[1] = new Waypoint(x-6*y,-12,0);
		waypoints[2] = new Waypoint(x-2*y, -12,0);
		waypoints[3] = new Waypoint(x, y-9, Pathfinder.d2r(90));
		if(position == RIGHT_POSITION)
			return mirrorWaypoints(waypoints);
		else
			return waypoints;
	}

	/*private Waypoint[] calculateHookPoints(double x, double y) {
		Waypoint[] waypoints = new Waypoint[3];
		waypoints[0] = new Waypoint(0, 0, 0);
		waypoints[1] = new Waypoint(x - Math.abs(y), 0, 0);
		if (y < 0) {
			waypoints[2] = new Waypoint(x, y, Pathfinder.d2r(-90));
		} else {
			waypoints[2] = new Waypoint(x, y, Pathfinder.d2r(90));
		}
		return waypoints;
	}*/

	private Waypoint[] calculateStraightPoints(double x) {
        return new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(x, 0, 0) };
	}

	/*private Waypoint[] calculateSwitchCurvePoints(double x, double y) {
		Waypoint[] waypoints = new Waypoint[3];
		waypoints[0] = new Waypoint(0, 0, 0);
		if (y < 0) {
			waypoints[1] = new Waypoint(x / 2, y / 2, Pathfinder.d2r(-45));
		} else {
			waypoints[1] = new Waypoint(x / 2, y / 2, Pathfinder.d2r(45));
		}
		waypoints[2] = new Waypoint(x, y, 0);
		return waypoints;
	}*/

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
		
		/*if (calculatedTrajectory == null) {
			System.out.println("Uh-Oh! Trajectory could not be generated!\n");
		}*/
        if (waypoints != null) {
            for (Waypoint waypoint : waypoints) {
                System.out.println(waypoint.x + " " + waypoint.y + " " + waypoint.angle);
            }
        }

		return calculatedTrajectory;
	}

	private Waypoint[] calculatePathWaypoints(String gameData, int robotPosition) {
		Waypoint[] returnWaypoints = null;
        switch (robotPosition) {
            case CENTER_POSITION:
				targetObject = SWITCH;
				if (gameData.charAt(0) == 'R') {
					targetSide = RIGHT;
				} else {
					targetSide = LEFT;
				}
				returnWaypoints = calculateCenterSwitchPoints(targetSide);
                break;
            case RIGHT_POSITION:
    			targetSide =RIGHT;
                if (!isStraightPathForced()) {
                    if (gameData.charAt(1) == 'R' && gameData.charAt(0) == 'R') {
                        if (isScalePreferredOverSwitch()) {
    						targetObject =SCALE;
    						returnWaypoints = calculateSideScaleHookPoints(robotPosition);
                            //returnWaypoints = calculateScalePoints(scaleEndPoint);
                        } else {
    						targetObject =SWITCH;
                        	returnWaypoints = calculateSideSwitchHookPoints(robotPosition);
                            //returnWaypoints = calculateHookPoints(hookEndPoint.x, hookEndPoint.y);
                        }
                    } else if (gameData.charAt(0) == 'R') {
						targetObject =SWITCH;
                        //returnWaypoints = calculateHookPoints(hookEndPoint.x, hookEndPoint.y);
                    	returnWaypoints = calculateSideSwitchHookPoints(robotPosition);
                    } else if (gameData.charAt(1) == 'R') {
                    	returnWaypoints = calculateSideScaleHookPoints(robotPosition);
                    	targetObject = SCALE;
                    } else {
    					targetObject =NONE;
                        returnWaypoints = calculateStraightPoints(straightEndPoint);
                    }
                } else {
					targetObject =NONE;
                    returnWaypoints = calculateStraightPoints(straightEndPoint);
                }
                break;
            case LEFT_POSITION:
    			targetSide =LEFT;
                if (!isStraightPathForced()) {
                    if (gameData.charAt(1) == 'L' && gameData.charAt(0) == 'L') {
                        if (isScalePreferredOverSwitch()) {
    						returnWaypoints = calculateSideScaleHookPoints(robotPosition);
                            //returnWaypoints = mirrorWaypoints(calculateScalePoints(scaleEndPoint));
    						targetObject =SCALE;
                        } else {
                        	returnWaypoints = calculateSideSwitchHookPoints(robotPosition);
                           //returnWaypoints = mirrorWaypoints(calculateHookPoints(hookEndPoint.x, hookEndPoint.y));
    						targetObject =SWITCH;
                        }
                    } else if (gameData.charAt(0) == 'L') {
						targetObject =SWITCH;
                    	returnWaypoints = calculateSideSwitchHookPoints(robotPosition);
                        //returnWaypoints = mirrorWaypoints(calculateHookPoints(hookEndPoint.x, hookEndPoint.y));
                    } else if (gameData.charAt(1) == 'L') {
                    	returnWaypoints = calculateSideScaleHookPoints(robotPosition);
                    	targetObject = SCALE;
                    } else {
                        returnWaypoints = calculateStraightPoints(straightEndPoint);
    					targetObject =NONE;
                    }
                } else {
                    returnWaypoints = calculateStraightPoints(straightEndPoint);
					targetObject =NONE;
                }
                break;
        }
        if(returnWaypoints != null) {
        	return waypointsInchesToMeters(returnWaypoints);
        } else {
        	return null;
        }
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

	private void putFMSDataOnDashboard(String data) {
		SmartDashboard.putString("FMS Data", data);
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
		PathData pathData = new PathData();
		
		pathData.time = timer.get();
		pathData.data[0] = 12 * (Robot.driveTrain.getLeftDistance());
		pathData.data[2] = 12 * (Robot.driveTrain.getRightDistance());
		Segment leftSegment = leftTrajectory.get(pathIndex);
		Segment rightSegment = rightTrajectory.get(pathIndex);
		pathData.data[1] = metersToInches(leftSegment.position);
		pathData.data[3] = metersToInches(rightSegment.position);
		pathData.data[4] = Robot.driveTrain.getHeading(); // Assuming the gyro is giving a value in degrees
		double th = Pathfinder.r2d(rightSegment.heading); // Should also be in degrees
		pathData.data[5] = th > 180 ? th - 360 : th; // convert to signed angle fixes problem:th 0->360 gh:-180->180
		pathDataList.add(pathData);
	}

	private static void publish(ArrayList<PathData> dataList, int traces) {
    	double info[] = new double[3];
    	int points = dataList.size();
    	info[0] = plotCount;
    	info[1] = traces;
    	info[2] = points;
    
    	System.out.println("Publishing Plot Data");
    	//table.putValue("NewPlot", NetworkTableValue.makeDoubleArray(info));
		table.putNumberArray("NewPlot"+ plotCount, info);

    	for (int i = 0; i < points; i++) {
    		PathData pathData = dataList.get(i);
			double data[] = new double[traces+2];
			data[0] = (double) i;
			data[1] = pathData.time;
			for(int j = 0; j < traces; j++) {
				data[j+2]=pathData.data[j];
			}
			table.putNumberArray("PlotData" + i, data);
		}
    	dataList.clear();
    	plotCount++;
    }
	public class PathData {
		static final int DATA_SIZE=6;
		double time = 0;
		double data[]=new double[DATA_SIZE];
	}

}
