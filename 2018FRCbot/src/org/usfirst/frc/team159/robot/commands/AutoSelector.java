package org.usfirst.frc.team159.robot.commands;

import org.usfirst.frc.team159.robot.Constants;
import org.usfirst.frc.team159.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelector extends CommandGroup implements Constants {
	private int targetObject = OBJECT_NONE;
	private int targetSide = POSITION_ILLEGAL;

	private int strategyOption = STRATEGY_OUR_SCALE;
	private int strategy = 0;

	public AutoSelector() {
		getStrategy();
	}

	private void showAutoTarget() {
		String which_object[] = { "Switch", "Scale", "Straight" };
		String which_side[] = { "Center", "Left", "Right" };
		String targetString = which_side[targetSide] + "-" + which_object[targetObject];
		SmartDashboard.putString("Target", targetString);
	}

	private int getSoftStrategy() {
		// TODO get from Dashboard
		return -1;
		// return strategyChooser.getSelected();
	}

	private int getHardStrategy() {
		// replace this with getHardwareSwitches in real robot code
		return -1;
	}

	private void getStrategy() {
		strategyOption = getSoftStrategy();
		setTargets();
		showAutoTarget();
	}

	private void setTargets() {
		String gameMessage = Robot.fmsData;
		boolean allGood = false;
		boolean allBad = false;

		if (gameMessage.length() > 0) {
			if (gameMessage.charAt(1) == 'L' && gameMessage.charAt(0) == 'L') {
				allGood = true;
			} else if (gameMessage.charAt(1) == 'R' && gameMessage.charAt(0) == 'R') {
				allBad = true;
			}

			if (Robot.robotPosition == POSITION_CENTER) {
				targetObject = OBJECT_SWITCH;
				if (gameMessage.charAt(0) == 'R') {
					targetSide = POSITION_RIGHT;
				} else {
					targetSide = POSITION_LEFT;
				}
			} else if (Robot.robotPosition == POSITION_RIGHT) {
				targetSide = POSITION_RIGHT;
				if (gameMessage.charAt(1) == 'R' && gameMessage.charAt(0) == 'L') {
					targetObject = OBJECT_SCALE;
				} else {
					targetObject = OBJECT_SWITCH;
				}
			} else if (Robot.robotPosition == POSITION_LEFT) {
				targetSide = POSITION_LEFT;
				if (gameMessage.charAt(1) == 'L' && gameMessage.charAt(0) == 'R') {
					targetObject = OBJECT_SCALE;
				} else {
					targetObject = OBJECT_SWITCH;
				}
			}

			switch (strategyOption) {
			case STRATEGY_TWO_CUBES:
				if (allGood) {
					targetObject = OBJECT_SCALE;
					break;
				} // intentional fall-thru
				strategyOption = STRATEGY_OPPOSITE_SCALE;
			case STRATEGY_OPPOSITE_SCALE:
				if (allBad) {
					targetObject = OBJECT_SCALE;
					targetSide = Robot.robotPosition == POSITION_RIGHT ? POSITION_LEFT : POSITION_RIGHT;
					break;
				} // intentional fall-thru
				strategyOption = STRATEGY_OUR_SCALE;
			case STRATEGY_OUR_SCALE:
				if (allGood || targetObject == OBJECT_SCALE) {
					targetObject = OBJECT_SCALE;
					break;
				} // intentional fall-thru
				strategyOption = STRATEGY_SWITCH_ONLY;
			case STRATEGY_SWITCH_ONLY:
				if (!(allGood || targetObject == OBJECT_SWITCH)) {
					targetObject = OBJECT_NONE;
					break;
				}
			case STRATEGY_STRAIGHT:
				targetObject = OBJECT_NONE;
				strategyOption = STRATEGY_STRAIGHT;
			}
		} else {
			System.out.println("AutoSelector: gameMessage is empty");
		}
	}

	private void addAutoCommand() {
		boolean mirror = (Robot.robotPosition == POSITION_LEFT);
		// addSequential(new SetGrabberState(HOLD, 0.1));
		switch (strategyOption) {
		case STRATEGY_GO_STRAIGHT:
			System.out.println("Go Straight");
			// autoCommand.addSequential(new DrivePath(STRATEGY_STRAIGHT ,false,false));
			break;
		case STRATEGY_CENTER_SWITCH:
			mirror = (targetSide == POSITION_RIGHT);
			System.out.println("Center Switch");
			addParallel(new SetElevator(SWITCH_DROP_HEIGHT, 2.0));
			addSequential(new DrivePath(CENTER_SWITCH, mirror, false));
			addSequential(new SetGrabberState(PUSH, 1.0));
			break;
		case STRATEGY_SAME_SWITCH:
			System.out.println("Same Switch");
			addParallel(new SetElevator(SWITCH_DROP_HEIGHT, 2.0));
			addSequential(new DrivePath(SAME_SWITCH, mirror, false));
			addSequential(new SetGrabberState(PUSH, 1.0));
			break;
		case STRATEGY_SIDE_TWO_CUBES:
			System.out.println("Two Cube Side");
			addAutoCommand(STRATEGY_OUR_SCALE); // reentrant call !
			addSequential(new SetElevator(0, 2.0)); // drop elevator and prepare to grab
			if (Robot.robotPosition == POSITION_RIGHT) // note: pathfinder can't turn in place or drive in reverse
				addSequential(new TurnToAngle(125.0, 3.0));
			else
				addSequential(new TurnToAngle(-125.0, 3.0));
			addSequential(new SetGrabberState(GRAB, 0.5));
			addSequential(new DrivePath(TWO_CUBE_SIDE, mirror, false));
			addSequential(new SetElevator(SWITCH_DROP_HEIGHT, 2.0));
			if (Robot.robotPosition == POSITION_RIGHT) // turn more toward center of switch
				addSequential(new TurnToAngle(25.0, 3.0));
			else
				addSequential(new TurnToAngle(-25.0, 3.0));
			addSequential(new SetGrabberState(PUSH, 1.0));
			break;
		case STRATEGY_CENTER_TWO_CUBES:
			System.out.println("Two Cube Center");
			addAutoCommand(CENTER_SWITCH); // place first cube (reentrant call !)
			// two-cube auto from center
			mirror = (targetSide == POSITION_LEFT); // inverted for backwards travel
			addSequential(new DrivePath(TWO_CUBE_CENTER, mirror, true)); // reverse s-turn from switch
			addParallel(new SetElevator(0, 2.0)); // set intake to grab cube
			addParallel(new SetGrabberState(OPEN, 0.5));
			addParallel(new SetGrabberState(GRAB, 0.5));
			addSequential(new DriveStraight(28, 0.4, 2.0, 0)); // drive forward and grab end cube
			addSequential(new SetGrabberState(CLOSE, 0.5));
			addSequential(new DriveStraight(-24, 0.4, 2.0, 0)); // back up
			addParallel(new SetElevator(SWITCH_DROP_HEIGHT, 2.0));
			addSequential(new DrivePath(TWO_CUBE_CENTER, !mirror, false)); // forward s-turn to switch
			addSequential(new SetGrabberState(PUSH, 1.0));

			break;
		case STRATEGY_SAME_SCALE:
			System.out.println("Same Scale");
			addParallel(new SetElevator(SWITCH_DROP_HEIGHT, 2.0));
			addSequential(new DrivePath(SAME_SCALE, mirror, false));
			addSequential(new SetElevator(SCALE_DROP_HEIGHT, 2.0));
			addSequential(new SetGrabberState(PUSH, 1.0));
			break;
		case STRATEGY_OTHER_SCALE:
			System.out.println("Other Scale");
			addParallel(new SetElevator(SWITCH_DROP_HEIGHT, 2.0));
			addSequential(new DrivePath(OTHER_SCALE, mirror, false));
			addSequential(new SetElevator(SCALE_DROP_HEIGHT, 2.0));
			addSequential(new SetGrabberState(PUSH, 1.0));
			break;
		}
	}
}
