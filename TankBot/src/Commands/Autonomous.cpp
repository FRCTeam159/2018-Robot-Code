#include "Autonomous.h"
#include "Commands/DriveStraight.h"
//Autonomous::Autonomous() {
//
//	AddParallel(new VisionUpdate());
//	AddSequential(new Delay(0.5));
//	AddSequential(new DriveToTarget());
//}
Autonomous::Autonomous() {

	AddSequential(new DriveStraight(60));
}
