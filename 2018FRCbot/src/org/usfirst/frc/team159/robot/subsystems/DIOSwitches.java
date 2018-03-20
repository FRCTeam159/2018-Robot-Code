package org.usfirst.frc.team159.robot.subsystems;

import org.usfirst.frc.team159.robot.Constants;
import org.usfirst.frc.team159.robot.Robot;
import org.usfirst.frc.team159.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
//	DigitalInput oppositeSide = new DigitalInput(ALLOW_OPPOSITE_CHANNEL);	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
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

    /*public void getPreferences() {
		//TODO complete this

//		Robot.oppositeSideAllowed = oppositeSide.get();
		
//		System.out.println("left: " + !leftPosition.get() + " center: " + !centerPosition.get() + " right: " + !rightPosition.get());
//		System.out.println("1: " + !strategyOne.get() + " 2: " + !strategyTwo.get() + " 3: " + !strategyThree.get() + " 4: " + !strategyFour.get());
		
		// values are inverted
		
		if(!leftPosition.get()) {
			Robot.robotPosition = POSITION_LEFT;
		} else if(!centerPosition.get()) {
			Robot.robotPosition = POSITION_CENTER;
		} else if(!rightPosition.get()) {
			Robot.robotPosition = POSITION_RIGHT;
		}
		
		int strategy = STRATEGY_STRAIGHT;
		
		if(!strategyFour.get()) {
			strategy = STRATEGY_TWO_CUBES;
		} else if(!strategyThree.get()) {
			strategy = STRATEGY_OPPOSITE_SCALE;
		} else if(!strategyTwo.get()) {
			strategy = STRATEGY_OUR_SCALE;
		} else if(!strategyOne.get()) {
			strategy = STRATEGY_SAME_SIDE_SWITCH;
		}
		
		Timer timer = new Timer();
		
		String gameMessage = DriverStation.getInstance().getGameSpecificMessage();
		while((gameMessage.equals("") || gameMessage == null) && timer.get() < 1) {
			gameMessage = DriverStation.getInstance().getGameSpecificMessage();
		}
		
		if(!leftPosition.get() || !rightPosition.get()) {
			switch(strategy) {
				case STRATEGY_TWO_CUBES:
					if(!leftPosition.get()) {
						if(gameMessage.startsWith("LL")) {
							break;
						}
					} else {
						if(gameMessage.startsWith("RR")) {
							break;
						}
					}
					strategy = STRATEGY_OPPOSITE_SCALE;
				case STRATEGY_OPPOSITE_SCALE:
					if(!leftPosition.get()) {
						if(gameMessage.startsWith("RR")) {
							break;
						}
					} else {
						if(gameMessage.startsWith("LL")) {
							break;
						}
					}
					strategy = STRATEGY_OUR_SCALE;
				case STRATEGY_OUR_SCALE:
					if(!leftPosition.get()) {
						if(gameMessage.length() > 0 && (gameMessage.charAt(0) == 'L' || gameMessage.charAt(1) == 'L')) {
							break;
						}
					} else {
						if(gameMessage.length() > 0 && (gameMessage.charAt(0) == 'R' || gameMessage.charAt(1) == 'R')) {
							break;
						}
					}
					strategy = STRATEGY_SAME_SIDE_SWITCH;
				case STRATEGY_SAME_SIDE_SWITCH:
					if(!leftPosition.get()) {
						if(gameMessage.startsWith("L")) {
							break;
						}
					} else {
						if(gameMessage.startsWith("R")) {
							break;
						}
					}
					strategy = STRATEGY_STRAIGHT;
				case STRATEGY_STRAIGHT:
					
					
				
			}
			
			if(strategy == STRATEGY_TWO_CUBES) {
				System.out.println("TWO_CUBES");
			} else if(strategy == STRATEGY_OPPOSITE_SCALE) {
				System.out.println("ALLOW_OPPOSITE_SIDE");
			} else if(strategy == STRATEGY_OUR_SCALE) {
				System.out.println("OUR_SIDE_ONLY");
			} else if(strategy == STRATEGY_SAME_SIDE_SWITCH) {
				System.out.println("SWITCH_ONLY");
			} else if(strategy == STRATEGY_STRAIGHT) {
				System.out.println("STRAIGHT");
			}
		}
	}*/
}

