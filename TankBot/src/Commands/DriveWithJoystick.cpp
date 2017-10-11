#include "DriveWithJoystick.h"
#include "Subsystems/DriveTrain.h"
#include "RobotMap.h"

#define TANK 1
#define ARCADE2 2
#define ARCADE3 3
#define CONTROLMODE ARCADE2

DriveWithJoystick::DriveWithJoystick()
{
	// Use Requires() here to declare subsystem dependencies
	Requires(driveTrain.get());
}

// Called just before this Command runs the first time
void DriveWithJoystick::Initialize()
{
	std::cout << "DriveWithJoystick::Initialize()" << std::endl;
}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystick::Execute()
{

	// Get axis values
	Joystick *stick = oi->GetJoystick();


	if (stick->GetRawButton(LOWGEAR_BUTTON)){
		driveTrain->SetLowGear();
	}
	else if(stick->GetRawButton(HIGHGEAR_BUTTON)){
		driveTrain->SetHighGear();
	}
	float yAxis=0,xAxis=0,zAxis=0;
#if CONTROLMODE == ARCADE2
	xAxis = stick-> GetRawAxis(1);
	yAxis = -stick-> GetRawAxis(4);
	driveTrain.get()->ArcadeDrive(xAxis, yAxis, false);
#elif CONTROLMODE == TANK
	yAxis = -stick-> GetRawAxis(5);
	xAxis = -stick-> GetRawAxis(1);
	driveTrain.get()->TankDrive(xAxis, yAxis);
#else // ARCADE3
	float yAxis = stick-> GetY();
	float xAxis = stick-> GetX();
	float zAxis = stick-> GetZ();
	// Run axis values through deadband
	yAxis = quadDeadband(.3, .4, yAxis);
	xAxis = quadDeadband(xMinThreshold, xMinOutput, xAxis);
	if(driveTrain->IsInLowGear()){ // if we are in low gear, decrease sensitivity
		zAxis = 0.70*quadDeadband(zMinThreshold, zMinOutput, zAxis);
	} else {
		zAxis = quadDeadband(zMinThreshold, zMinOutput, zAxis);
	}
	driveTrain.get()->CustomArcade(xAxis, yAxis, zAxis,true);
#endif

}

// Make this return true when this Command no longer needs to run execute()
bool DriveWithJoystick::IsFinished()
{
	return false;
}

// Called once after isFinished returns true
void DriveWithJoystick::End()
{
	std::cout << "DriveWithJoystick Finished" << std::endl;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveWithJoystick::Interrupted()
{
	End();
}
float DriveWithJoystick::quadDeadband(float minThreshold, float minOutput, float input)
{
	if (input > minThreshold) {
		return ((((1 - minOutput) // 1 - minOutput/(1-minThreshold)^2 * (input-minThreshold)^2 + minOutput
				/ ((1 - minThreshold)* (1 - minThreshold)))
				* ((input - minThreshold)* (input - minThreshold)))
				+ minOutput);
	} else {
		if (input < (-1 * minThreshold)) {
			return (((minOutput - 1) // minOutput - 1/(minThreshold - 1)^2 * (minThreshold + input)^2 - minOutput
					/ ((minThreshold - 1)* (minThreshold - 1)))
					* ((minThreshold + input)* (minThreshold + input)))
					- minOutput;
		}

		else {
			return 0;
		}
	}
}
