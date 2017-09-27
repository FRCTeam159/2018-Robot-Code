#ifndef OI_H
#define OI_H

#include "WPILib.h"

class OI {
public:
	Joystick *stick;
	OI();
	Joystick *GetJoystick();
};

#endif  // OI_H
