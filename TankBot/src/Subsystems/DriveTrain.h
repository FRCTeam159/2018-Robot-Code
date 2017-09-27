#ifndef DriveTrain_H
#define DriveTrain_H
#include "WPILib.h"
#include "Commands/Subsystem.h"
#include "CANTalon.h"
#include "ErrorBase.h"
#include "MotorSafety.h"
#include "MotorSafetyHelper.h"
#include <ADXRS450_Gyro.h>

using namespace frc;

class DriveTrain: public Subsystem, public MotorSafety {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	CANTalon frontLeft;
	CANTalon frontRight;
	CANTalon backLeft;
	CANTalon backRight;
	DoubleSolenoid *gearPneumatic;
	ADXRS450_Gyro gyro;
	bool inlowgear=false;
	void InitDrive();
	void CustomArcade(float xAxis, float yAxis, float zAxis, bool squaredInputs);
	float coerce(float min, float max, float x);
	void Publish(bool);
public:
	DriveTrain();
	void Drive(float xAxis, float yAxis, float zAxis);
	void InitDefaultCommand();
	void SetLowGear();
	void SetHighGear();
	bool IsInLowGear();
	void DisableDrive();
	void EnableDrive();
	void TankDrive(float xAxis, float yAxis);

	// required MotorSafety functions
	std::unique_ptr<MotorSafetyHelper> m_safetyHelper;
	void SetExpiration(double timeout) override;
	double GetExpiration() const override;
	bool IsAlive() const override;
	void StopMotor() override;
	bool IsSafetyEnabled() const override;
	void SetSafetyEnabled(bool enabled) override;
	void GetDescription(std::ostringstream& desc) const override;
	void Enable();
	int sign;

	//Anonomous code
	double GetDistance();//DriveStraight code
	double GetLeftDistance();
	double GetRightDistance();

	void Reset();


	double GetHeading();
};

#endif
