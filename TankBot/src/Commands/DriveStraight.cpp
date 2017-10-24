#include "DriveStraight.h"

// magic P = 0.9, cycles per period = 6
#define P 0.54
#define I 0.18
#define D 0.405
#define TOL 0.1

static int cycle_count = 0;

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
    frc::SmartDashboard::PutNumber("Target",distance);
}

// Called just before this Command runs the first time
void DriveStraight::Initialize() {
	cycle_count = 0;
	double p = frc::SmartDashboard::GetNumber("P",P);
	double i = frc::SmartDashboard::GetNumber("I",I);
	double d = frc::SmartDashboard::GetNumber("D",D);
	tolerance=frc::SmartDashboard::GetNumber("TOL",tolerance);
	distance = frc::SmartDashboard::GetNumber("Target",distance);
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
	if(new_target && last_ontarget){
		return true;
	}
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

//#define DEBUG_COMMAND
#define PRINT_CYCLES

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
#ifdef PRINT_CYCLES
    if(cycle_count > 0){
    	std::cout << cycle_count << " "<< d << std::endl;
    }
    cycle_count++;
#endif
	driveTrain->TankDrive(d,d);
}
