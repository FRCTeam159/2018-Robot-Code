#include "DriveWithJoystick.h"
#include "Subsystems/DriveTrain.h"
#include "RobotMap.h"

#define USE_GAMEPAD

DriveWithJoystick::DriveWithJoystick()
{
	// Use Requires() here to declare subsystem dependencies
	Requires(driveTrain.get());
}

// Called just before this Command runs the first time
void DriveWithJoystick::Initialize()
{
	std::cout << "DriveWithJoystick::Initialize()" << std::endl;

	frc::SmartDashboard::PutNumber("xMinThreshold", 0.7);
	frc::SmartDashboard::PutNumber("xMinOutput", 0.8);

	frc::SmartDashboard::PutNumber("zMinThreshold", 0.4);
	frc::SmartDashboard::PutNumber("zMinOutput", 0.6);
}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystick::Execute()
{
	double xMinThreshold = SmartDashboard::GetNumber("xMinThreshold",0.7);
	double xMinOutput = SmartDashboard::GetNumber("xMinOutput",0.8);

	double zMinThreshold = SmartDashboard::GetNumber("zMinThreshold",0.4);
	double zMinOutput = SmartDashboard::GetNumber("zMinOutput",0.6);

	// Get axis values
	Joystick *stick = oi->GetJoystick();


	if (stick->GetRawButton(LOWGEAR_BUTTON)){
		driveTrain->SetLowGear();
	}
	else if(stick->GetRawButton(HIGHGEAR_BUTTON)){
		driveTrain->SetHighGear();
	}
#ifdef USE_GAMEPAD
	float yAxis = -stick-> GetRawAxis(1);
	float xAxis = -stick-> GetRawAxis(5);
	driveTrain.get()->TankDrive(xAxis, yAxis);
#else
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
	cout << xAxis << endl;
	driveTrain.get()->Drive(xAxis, yAxis, zAxis);
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
