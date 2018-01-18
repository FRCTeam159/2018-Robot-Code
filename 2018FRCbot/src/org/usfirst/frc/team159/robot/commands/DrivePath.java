package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 *
 */
public class DrivePath extends Command {
	double TIME_STEP = 0.02;
	double MAX_VEL = 2.15;  //2.75 m/s measured, but reduced to avoid exceeding max on outside wheels when turning
	double MAX_ACC = 14.75;
	double MAX_JRK = 160.0;
	double KP = 0.5;
	double KI = 0.0;
	double KD = 0.0;
	double KV = 1.0/MAX_VEL;
	double KA = 0.0;
	double GFACT = 5.0;
	double wheelbase = metersPerInch(26);
	Trajectory trajectory;
	Trajectory leftTrajectory;
	Trajectory rightTrajectory;
	DistanceFollower leftFollower;
	DistanceFollower rightFollower;
	Trajectory.Config config;
	TankModifier modifier;
	Timer timer;
	static public boolean plotPath = true;
	static public boolean plotTrajectory = false;
	static public boolean useGyro = false;
	static public boolean debugCommand = true;

	double runtime=0;
	Waypoint[] straightPoints = new Waypoint[] {		 
		    new Waypoint(0, 0, 0),
		    new Waypoint(2, 0, 0),
		};
	Waypoint[] hookPoints = new Waypoint[] {		 
		    new Waypoint(0, 0, 0),
		    new Waypoint(0.5, 0, 0),
		    new Waypoint(1.5, 0, Pathfinder.d2r(45)),
		    new Waypoint(2, 2, 0)
		};

    public DrivePath() {
		requires(Robot.driveTrain);
		timer=new Timer();
    	timer.start();
    	timer.reset();
    	
    	System.out.println(System.getProperty("java.library.path"));
    	System.out.println(wheelbase);

		config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, 
				TIME_STEP, MAX_VEL, MAX_ACC,MAX_JRK);
		trajectory = Pathfinder.generate(hookPoints, config);
		
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
    	double herr = th - gh;
    	if(useGyro) 
    		turn = GFACT * (-1.0/180.0) * herr;
    	if(debugCommand) 
    		System.out.format("%f %f %f %f %f\n", timer.get(), ld, rd,l+turn,r-turn);
    	//    		System.out.format("%f %f %f %f %f %f %f %f\n", mytimer.get(), ld, rd,gh,th,herr,l+turn,r-turn);

    	Robot.driveTrain.tankDrive(l+turn,r-turn);
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
