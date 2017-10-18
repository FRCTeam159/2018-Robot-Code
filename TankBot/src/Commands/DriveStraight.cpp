#include "DriveStraight.h"

#define P 1.0
#define I 0.0
#define D 0.0
#define TOL 0.1

DriveStraight::DriveStraight(double d)  : CommandBase("DriveStraight"),
	pid(P,I,D,this,this)
{
	Requires(driveTrain.get());
	distance=d;
	tolerance=TOL;
    std::cout << "new DriveStraight("<<d<<")"<< std::endl;
    frc::SmartDashboard::PutNumber("P",P);
    frc::SmartDashboard::PutNumber("I",I);
    frc::SmartDashboard::PutNumber("D",D);
    frc::SmartDashboard::PutNumber("TOL",TOL);
}

// Called just before this Command runs the first time
void DriveStraight::Initialize() {
	double p = frc::SmartDashboard::GetNumber("P",P);
	double i = frc::SmartDashboard::GetNumber("I",I);
	double d = frc::SmartDashboard::GetNumber("D",D);
	tolerance=frc::SmartDashboard::GetNumber("TOL",TOL);
	last_ontarget=false;
	pid.SetPID(p,i,d);
	driveTrain->Reset();
  	pid.Reset();
	pid.SetSetpoint(distance);
	pid.SetAbsoluteTolerance(tolerance);
	pid.Enable();
	//pid.SetToleranceBuffer(5);
	driveTrain->Enable();
	std::cout << "DriveStraight Started .."<< std::endl;
}

// Called repeatedly when this Command is scheduled to run
void DriveStraight::Execute() {
}

// Make this return true when this Command n o longer needs to run execute()
bool DriveStraight::IsFinished() {
	bool new_target=pid.OnTarget();
	return new_target && last_ontarget;
	last_ontarget=new_target;
	return false;
}

// Called once after isFinished returns true
void DriveStraight::End() {
	pid.Disable();
    std::cout << "DriveStraight End" << std::endl;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveStraight::Interrupted() {
	End();
}

#define DEBUG_COMMAND

double DriveStraight::PIDGet() {
	double s=driveTrain->GetDistance();
#ifdef DEBUG_COMMAND
    std::cout << "DriveStraight::PIDGet("<<s<<")"<< std::endl;
#endif
	return s;
}

void DriveStraight::PIDWrite(double d) {
#ifdef DEBUG_COMMAND
    std::cout << "DriveStraight::PIDWrite("<<d<<")"<< std::endl;
#endif
	driveTrain->TankDrive(d,d);
}
