#ifndef DriveStraight_H
#define DriveStraight_H

#include "../CommandBase.h"

class DriveStraight : public CommandBase , public PIDSource, public PIDOutput{
	PIDController pid;
	double distance;
	double tolerance;
	bool last_ontarget=false;
public:
	DriveStraight(double d);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
    double PIDGet();
    void PIDWrite(double d);

};

#endif  // DriveStraight_H
