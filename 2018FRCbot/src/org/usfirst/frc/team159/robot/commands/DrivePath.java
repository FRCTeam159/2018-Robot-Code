package org.usfirst.frc.team159.robot.commands;

import java.util.Random;

import org.usfirst.frc.team159.robot.PhysicalConstants;
import org.usfirst.frc.team159.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
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
	double TIME_STEP = 0.02;
	double MAX_VEL = 5.5 * 0.3048; //1.8
	double MAX_ACC = 8.6 * 0.3048; //14
	double MAX_JRK = 64; //116
	//double MAX_VEL = 2.15;  //2.75 m/s measured, but reduced to avoid exceeding max on outside wheels when turning
	//double MAX_ACC = 14.75;
	//double MAX_JRK = 160.0;
	double KP = 0.5;
	double KI = 0.0;
	double KD = 0.0;
	double KV = 1.0/MAX_VEL;
	double KA = 0.0;
	double GFACT = 2.0;
	double wheelbase = metersPerInch(26);
	Trajectory trajectory;
	Trajectory leftTrajectory;
	Trajectory rightTrajectory;
	DistanceFollower leftFollower;
	DistanceFollower rightFollower;
	Trajectory.Config config;
	TankModifier modifier;
	Timer timer;
	static public boolean plotPath = false;
	static public boolean plotTrajectory = false;
	static public boolean useGyro = false;
	static public boolean debugCommand = false;
	private static final boolean debugPath = true;
	
	private int pathIndex = 0;
	
	private static final int LEFT_POSITION = 0;
	private static final int CENTER_POSITION = 1;
	private static final int RIGHT_POSITION = 2;

	double runtime=0;
	private static final Double[] hookEndPoint = new Double[] {129.5, -32.125};
	private static final double straightEndPoint = 78;
	
	private static Waypoint[] straightPoints = new Waypoint[] {		 
		    new Waypoint(0, 0, 0),
		    new Waypoint(78, 0, 0),
	};
	private static Waypoint[] hookPoints = new Waypoint[] {		 
		    new Waypoint(0, 0, 0),
		    new Waypoint(SWITCH_HOOK_TURN_X, 0, 0),
		    new Waypoint(SWITCH_HOOK_TURN_X, SWITCH_HOOK_TURN_Y, Pathfinder.d2r(45)),
		    new Waypoint(ROBOT_TO_SWITCH_CENTER, HOOK_Y_DISTANCE, Pathfinder.d2r(90))
	};
	private static Waypoint[] switchCurvePoints = {
		new Waypoint(0, 0, 0),
		new Waypoint(ROBOT_CENTER_X_TURN_POINT, ROBOT_CENTER_Y_TURN_POINT, Pathfinder.d2r(45)),
		new Waypoint(ROBOT_TO_SWITCH, SWITCH_CENTER_TO_PLATE_EDGE, 0)
	};
	private static Waypoint[] scalePoints = {
		new Waypoint(0, 0, 0),
		new Waypoint(999999999, 0, 0),
	};
	
    public DrivePath() {
    	SendableChooser<Integer> robotPosition = (SendableChooser<Integer>) SmartDashboard.getData("Position");
    	String gameMessage = DriverStation.getInstance().getGameSpecificMessage();
    	if(gameMessage.equals("")) {
    		gameMessage = generateFMSData();
    	}
    	SmartDashboard.putString("FMS Data", gameMessage);
    	
		requires(Robot.driveTrain);
		timer=new Timer();
    	timer.start();
    	timer.reset();
    	
    	System.out.println(System.getProperty("java.library.path"));
    	System.out.println(wheelbase);

		config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, 
				TIME_STEP, MAX_VEL, MAX_ACC,MAX_JRK);
		//trajectory = Pathfinder.generate(calculatePath(gameMessage, robotPosition.getSelected()), config);
		Waypoint[] waypoints = calculateHookPoints(ROBOT_TO_SWITCH_CENTER, 48);
		for(Waypoint waypoint : waypoints) {
			System.out.println(waypoint.x + " " + waypoint.y + " " + waypoint.angle);
		}
		trajectory = Pathfinder.generate(waypointsInchesToMeters(waypoints), config);
		
		if(trajectory == null) {
			System.out.println("Uh-Oh! Trajectory could not be generated!\n");
			return;
		}
		runtime = trajectory.length()*TIME_STEP;
		// Create the Modifier Object
		TankModifier modifier = new TankModifier(trajectory);

		// Generate the Left and Right trajectories using the original trajectory
		// as the center
		modifier.modify(wheelbase);

		leftTrajectory  = modifier.getLeftTrajectory();       // Get the Left Side
		rightTrajectory = modifier.getRightTrajectory();      // Get the Right Side
		Segment seg=leftTrajectory.segments[0];
		
		leftFollower=new DistanceFollower(leftTrajectory);
		leftFollower.configurePIDVA(KP, KI, KD, KV, KA);
		rightFollower=new DistanceFollower(rightTrajectory);
		rightFollower.configurePIDVA(KP, KI, KD, KV, KA);
	    System.out.format("trajectory length:%d runtime:%f calctime:%f\n", trajectory.length(),runtime,timer.get());

		if(plotPath) {
			double t=0;
			for (int i = 0; i < trajectory.length(); i++) {
			    Segment s = trajectory.get(i);
			    System.out.format("%f %f %f %f %f %f \n", t, s.x, s.y,s.velocity, s.acceleration,s.heading);
			    t+=s.dt;
			}
		}
		if(plotTrajectory) {
			double t=0;
			for (int i = 0; i < trajectory.length(); i++) {
			    Segment s = trajectory.get(i);
			    Segment l = leftTrajectory.get(i);
			    Segment r = rightTrajectory.get(i);
			    System.out.format("%f %f %f %f %f\n", t, l.x,l.y,r.x,r.y);
			    t+=s.dt;
			}
		}
    }
    
    private Waypoint[] calculateHookPoints(double x, double y) { // 129.5, 32.125
    	Waypoint[] waypoints = new Waypoint[4];
    	waypoints[0] = new Waypoint(0, 0, 0);
		waypoints[1] = new Waypoint(x - Math.abs(y), 0, 0);
    	if(y < 0) {
    		waypoints[2] = new Waypoint(x - Math.abs(y/2), y + Math.abs(y/2), Pathfinder.d2r(-45));
    		waypoints[3] = new Waypoint(x, y, Pathfinder.d2r(90));
    	} else {
    		waypoints[2] = new Waypoint(x - (y/2), y - (y/2), Pathfinder.d2r(45));
    		waypoints[3] = new Waypoint(x, y, Pathfinder.d2r(-90));
    	}
    	return waypoints;
    }
    
    private Waypoint[] calculateStraightPoints(double x) {
    	return new Waypoint[] {new Waypoint(0, 0, 0), new Waypoint(x, 0, 0)};
    }
    
    private Waypoint[] mirrorWaypoints(Waypoint[] waypoints) {
    	Waypoint[] newWaypoints = new Waypoint[waypoints.length];
		for(int i = 0; i < waypoints.length; i++) {
			newWaypoints[i] = new Waypoint(waypoints[i].x, -waypoints[i].y, -waypoints[i].angle);
		}
		return newWaypoints;
    }
    
    private Waypoint[] waypointsInchesToMeters(Waypoint[] waypoints) {
    	Waypoint[] newWaypoints = new Waypoint[waypoints.length];
    	for(int i = 0; i < waypoints.length; i++) {
    		newWaypoints[i] = new Waypoint(inchesToMeters(waypoints[i].x), inchesToMeters(waypoints[i].y), waypoints[i].angle);
    	}
    	return newWaypoints;
    }
    
    private boolean isScalePreferredOverSwitch() {
    	return SmartDashboard.getBoolean("Prefer Scale", true);
    }
    
    private boolean isStraightPathForced() {
    	return SmartDashboard.getBoolean("Force Straight Path", false);
    }
    
    private Waypoint[] calculatePath(String gameData, int robotPosition) {
    	Waypoint[] returnWaypoints = null;
    	if(robotPosition == CENTER_POSITION) {
    		if(gameData.charAt(0) == 'R') {
    			returnWaypoints = switchCurvePoints;
    		} else {
    			returnWaypoints = mirrorWaypoints(switchCurvePoints);
    		}
    	} else if(robotPosition == RIGHT_POSITION) {
    		if(!isStraightPathForced()) {
	    		if(gameData.charAt(1) == 'R' && gameData.charAt(0) == 'R') {
	    			if(isScalePreferredOverSwitch()) {
	    				returnWaypoints = scalePoints;
	    			} else {
	    				returnWaypoints = calculateHookPoints(hookEndPoint[0], hookEndPoint[1]);
	    			}
	    		}
		    	if(gameData.charAt(0) == 'R') {
		    		returnWaypoints = calculateHookPoints(hookEndPoint[0], hookEndPoint[1]);
		    	} else {
		    		returnWaypoints = calculateStraightPoints(straightEndPoint);
		    	}
    		} else {
    			returnWaypoints = calculateStraightPoints(straightEndPoint);
    		}
    	} else if(robotPosition == LEFT_POSITION) {
    		if(!isStraightPathForced()) {
	    		if(gameData.charAt(1) == 'L' && gameData.charAt(0) == 'L') {
	    			if(isScalePreferredOverSwitch()) {
	    				returnWaypoints = mirrorWaypoints(scalePoints);
	    			} else {
	    				returnWaypoints = mirrorWaypoints(calculateHookPoints(hookEndPoint[0], hookEndPoint[1]));
	    			}
	    		}
		    	if(gameData.charAt(0) == 'L') {
		    		returnWaypoints = mirrorWaypoints(calculateHookPoints(hookEndPoint[0], hookEndPoint[1]));
		    	} else {
		    		returnWaypoints = calculateStraightPoints(straightEndPoint);
		    	}
    		} else {
    			returnWaypoints = calculateStraightPoints(straightEndPoint);
    		}
    	}
    	return waypointsInchesToMeters(returnWaypoints);
    }
    
    private String generateFMSData() {
    	Random random = new Random();
    	String data = "";
    	while(data.length() < 3) {
    		if(random.nextInt(2) == 0) {
    			data += "R";
    		} else {
    			data += "L";
    		}
    	}
    	return data;
	}
    
    private double inchesToMeters(double inches) {
    	return inches * 0.0254;
    }
    
    private double metersToInches(double meters) {
    	return meters / 0.0254;
    }
    
    private void debugPathError() {
     	double leftDistance = 12*(Robot.driveTrain.getLeftDistance());
    	double rightDistance = 12*(Robot.driveTrain.getRightDistance());
    	Segment segment = leftTrajectory.segments[pathIndex];
    	double leftTarget = metersToInches(segment.position);
    	segment = rightTrajectory.segments[pathIndex];
    	double rightTarget = metersToInches(segment.position);
    	double headingTarget = Pathfinder.r2d(segment.heading);
    	double currentHeading = Robot.driveTrain.getHeading();
    	
    	System.out.format("%f %f %f %f %f\n", timer.get(), leftDistance, leftTarget, rightDistance, rightTarget);
    	
    	pathIndex++;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(trajectory==null)
    		return;
		System.out.println("DrivePath.initialize()");
    	leftFollower.reset();
    	rightFollower.reset();
    	Robot.driveTrain.reset();
    	timer.start();
    	timer.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(trajectory==null)
    		return;
    	double ld=feetToMeters(Robot.driveTrain.getLeftDistance());
    	double rd=feetToMeters(Robot.driveTrain.getRightDistance());
    	double l = leftFollower.calculate(ld);
    	double r = rightFollower.calculate(rd);
 
    	double turn=0;

    	double gh = Robot.driveTrain.getHeading();    // Assuming the gyro is giving a value in degrees
    	double th = Pathfinder.r2d(leftFollower.getHeading());  // Should also be in degrees
    	
    	th = th > 180? 360 - th : th;
    	double herr = th - gh;
    	if(useGyro) 
    		turn = GFACT * (-1.0/180.0) * herr;
    	if(debugCommand) {
    		System.out.format("%f %f %f %f %f %f %f\n", timer.get(), ld, rd,th,gh,r+turn,l-turn);
    	}
    	
    	if(debugPath) {
    		debugPathError();
    	}
    	//    		System.out.format("%f %f %f %f %f %f %f %f\n", mytimer.get(), ld, rd,gh,th,herr,l+turn,r-turn);

    	Robot.driveTrain.tankDrive(r+turn,l-turn);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(trajectory==null)
    		return true;
    	if(timer.get()-runtime>0.5)
    		return true;
    	if(leftFollower.isFinished() && rightFollower.isFinished())
    		return true;

        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
		System.out.println("DrivePath.end()");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
    double feetToMeters(double feet) {
    	return 2.54*12*feet/100;
    }
    double metersPerInch(double inches) {
    	return 2.54*inches/100;
    }
}
