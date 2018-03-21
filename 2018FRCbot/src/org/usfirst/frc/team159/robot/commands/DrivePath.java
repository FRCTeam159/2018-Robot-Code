package org.usfirst.frc.team159.robot.commands;

import java.util.ArrayList;
import java.util.Random;

import org.usfirst.frc.team159.robot.PhysicalConstants;
import org.usfirst.frc.team159.robot.RobotMap;

import org.usfirst.frc.team159.robot.Robot;
import org.usfirst.frc.team159.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.DigitalInput;
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
public class DrivePath extends Command implements PhysicalConstants, RobotMap {

	private static final boolean forcedStraight = false;
	
	private static final double TIME_STEP = 0.02;
	private static final double wheelbase = inchesToMeters(26);
	private Trajectory trajectory;
	private Trajectory leftTrajectory;
	private Trajectory rightTrajectory;
	private DistanceFollower leftFollower;
	private DistanceFollower rightFollower;
	private static final boolean printCalculatedTrajectory = false;
	private static final boolean printCalculatedPath = false;
	private static boolean useGyro = false;
	private static final boolean debugCommand = false;
	private static final boolean printPath = false;

	//	private static int plotId = 0;
	private int pathIndex = 0;

	private static final int SWITCH = 0;
	private static final int SCALE = 1;
	private static final int NONE = 2;

	private static final int LEFT = 0;
	private static final int RIGHT = 1;

	private int targetObject = SCALE;
	private int targetSide = LEFT;
	
	private String whichObject[]= {"Switch","Scale","Straight"};
	private String whichSide[]= {"Left","Right"};
	private String whichPosition[] = {"Left", "Center", "Right", "ILLEGAL"};

	private Timer timer = new Timer();
	private Timer pushTimer = new Timer();
	private boolean pushing = false;
	private final double runtime;
	private double pushTime = 1;

  double last_heading = 0;
	private static final double straightEndPoint = ROBOT_TO_SWITCH;
	
	private static int plotCount = 0;

	private ArrayList<PathData> pathDataList = new ArrayList<>();
	//static NetworkTableInstance ti=NetworkTableInstance.getDefault();	
	//static NetworkTable table=ti.getTable("datatable");
	//TODO use non deprecated class (edu.wpi.first.networktables.NetworkTable)
	private static NetworkTable table = NetworkTable.getTable("datatable");

	private String targetString = whichSide[targetSide] + " " + whichObject[targetObject];
	
	private int robotPosition = ILLEGAL_POSITION;
	

	DrivePath() {
		requires(Robot.driveTrain);
		requires(Robot.elevator);
		
		timer.start();
		timer.reset();		
		
		robotPosition=Robot.robotPosition;
		System.out.println("Position="+robotPosition);

		timer.start();
		timer.reset();
		
    double MAX_VEL = Robot.MAX_VEL;
    double MAX_ACC = Robot.MAX_ACC;
    double MAX_JRK = Robot.MAX_JRK;
    double KP = Robot.KP;
    double KI = 0.0;
    double KD = Robot.KP;
    double KV = 1.0 / MAX_VEL;
    double KA = 0.0;
		
//		String gameMessage = DriverStation.getInstance().getGameSpecificMessage();
//		while((gameMessage.equals("") || gameMessage == null) && timer.get() < 1) {
//			gameMessage = DriverStation.getInstance().getGameSpecificMessage();
//		}
//
//		putFMSDataOnDashboard(gameMessage);
    
    String gameMessage=Robot.fmsData;
    System.out.println("FMS="+gameMessage);

		double maxVelocity = MAX_VEL;
		double maxAcceleration = MAX_ACC;
		double maxJerk = MAX_JRK;
		
		trajectory = calculateTrajectory(gameMessage, robotPosition, maxVelocity, maxAcceleration, maxJerk);
		
		if(trajectory == null) {
			SmartDashboard.putString("Target", "ERROR");
			runtime = 0;
			return;
		}
		
		targetString = "Position:"+whichPosition[robotPosition] + " Target:" + whichSide[targetSide] + " - " + whichObject[targetObject];
		SmartDashboard.putString("Target", targetString);
		
		// Create the Modifier Object
		TankModifier modifier = new TankModifier(trajectory);
		modifier.modify(wheelbase);
		runtime = trajectory.length() * TIME_STEP;

		// Generate the Left and Right trajectories using the original trajectory as the center

		leftTrajectory = modifier.getLeftTrajectory(); // Get the Left Side
		rightTrajectory = modifier.getRightTrajectory(); // Get the Right Side
		
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

		Robot.elevator.setElevatorTarget(Elevator.SWITCH_HEIGHT);
		
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

		double gh = Robot.driveTrain.getHeading(); // Assuming the gyro is giving a value in degrees
		gh = unwrap(last_heading, gh);
		
		double th = Pathfinder.r2d(leftFollower.getHeading()); // Should also be in degrees

		th = th > 180 ? th-360 : th;
		double headingError = th - gh;
		if (Robot.useGyro) {
			turn = Robot.GFACT * (-1.0 / 180.0) * headingError;
		}
		
		double lval = leftPower - turn;
		double rval = rightPower + turn;
		
		lval *= Robot.scale;
		rval *= Robot.scale;

		if (debugCommand) {
			System.out.format("%f %f %f %f %f %f %f\n", timer.get(), leftDistance, rightDistance, th, gh, rval, lval);
		}

		if (printPath) {
			debugPathError();
		}
		if (publishPathAllowed()) {
			addPlotData();
		}		
		pathIndex++;
		last_heading = gh;
		// this is reversed because we found it to be reversed, don't change unless you know what you're doing
		Robot.driveTrain.tankDrive(lval, rval);
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
			else if(targetObject == SWITCH) {
				pushing = true;
				pushTimer.reset();
				pushTimer.start();
			}
			else
				return true;
				
		} else if (pushTimer.get() - runtime > 2) {
			System.out.println("DrivePath Timeout Expired");
			return true;
		}		
        return false;
    }

	// Called once after isFinished returns true
  protected void end() {
    printEndMessage();
    if (publishPathAllowed()) {
      publish(pathDataList, 6);
    }
  }

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

  double unwrap(double previous_angle, double new_angle) {
    double d = new_angle - previous_angle;
    d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
    return previous_angle + d;
  }

	private boolean publishPathAllowed(){
	    return SmartDashboard.getBoolean("Publish Path", false);
    }

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

	private Waypoint[] calculateStraightPoints(double x) {
	  return new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(x, 0, 0) };
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
	
  private Trajectory calculateTrajectory(String gameData, int robotPosition, double maxVelocity, double maxAcceleration,
      double maxJerk) {
    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST,
        TIME_STEP, maxVelocity, maxAcceleration, maxJerk);

    Waypoint[] waypoints = calculatePathWaypoints(gameData, robotPosition);
    if (waypoints == null) {
      return null;
    }
    Trajectory calculatedTrajectory = Pathfinder.generate(waypoints, config);
    /*
     * if (calculatedTrajectory == null) {
     * System.out.println("Uh-Oh! Trajectory could not be generated!\n"); }
     */
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
					} else {
						targetObject =SWITCH;
						returnWaypoints = calculateSideSwitchHookPoints(robotPosition);
					}
				} else if (gameData.charAt(0) == 'R') {
					targetObject =SWITCH;
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
						targetObject =SCALE;
					} else {
						returnWaypoints = calculateSideSwitchHookPoints(robotPosition);
						targetObject =SWITCH;
					}
				} else if (gameData.charAt(0) == 'L') {
					targetObject =SWITCH;
					returnWaypoints = calculateSideSwitchHookPoints(robotPosition);
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

	private boolean isStraightPathForced() {
    return forcedStraight;
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

//	private void putFMSDataOnDashboard(String data) {
//		SmartDashboard.putString("FMS Data", data);
//	}
//	
//	private double getNumberOnDashboard(String name, double defaultValue) {
//		return SmartDashboard.getNumber(name, defaultValue);
//	}
	
	private void printInitializeMessage() {
		System.out.println("DrivePath.initialize()");
	}
	
	private void printEndMessage() {
		System.out.println("DrivePath.end()");
	}

	private boolean isScalePreferredOverSwitch() {
		return Robot.preferScale;
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
