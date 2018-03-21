package org.usfirst.frc.team159.robot.subsystems;

import org.usfirst.frc.team159.robot.Constants;
import org.usfirst.frc.team159.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DIOSwitches extends Subsystem implements RobotMap, Constants {
	DigitalInput strategyOne = new DigitalInput(0);
	DigitalInput strategyTwo = new DigitalInput(1);
	DigitalInput strategyThree = new DigitalInput(2);
	DigitalInput strategyFour = new DigitalInput(3);
	
	DigitalInput leftPosition = new DigitalInput(LEFT_POSITION_CHANNEL);
	DigitalInput centerPosition = new DigitalInput(CENTER_POSITION_CHANNEL);
	DigitalInput rightPosition = new DigitalInput(RIGHT_POSITION_CHANNEL);
	
	int position=-1;
  int strategy=-1;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public int getPosition() {
      if(!leftPosition.get()) {
        position = POSITION_LEFT;
      } else if(!centerPosition.get()) {
        position = POSITION_CENTER;
      } else if(!rightPosition.get()) {
        position = POSITION_RIGHT;
      }
      return position;
    }
    public int getStrategy() {
      
      if(!strategyFour.get()) {
        strategy = STRATEGY_TWO_CUBES;
      } else if(!strategyThree.get()) {
        strategy = STRATEGY_OPPOSITE_SCALE;
      } else if(!strategyTwo.get()) {
        strategy = STRATEGY_SAME_SIDE_SCALE;
      } else if(!strategyOne.get()) {
        strategy = STRATEGY_SAME_SIDE_SWITCH;
      }
      return strategy;
    }

 
}

