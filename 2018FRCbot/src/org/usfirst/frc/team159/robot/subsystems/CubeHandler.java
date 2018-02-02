package org.usfirst.frc.team159.robot.subsystems;

import org.usfirst.frc.team159.robot.RobotMap;
import org.usfirst.frc.team159.robot.commands.CubeCommands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CubeHandler extends Subsystem {

	// Put methods for controlling this subsystem here. Call these from Commands.
	private WPI_TalonSRX leftIntakeMotor;
	private WPI_TalonSRX rightIntakeMotor;

	private static final double intakePower = 0.5;
	private static final double outputPower = 0.25;

	private boolean droppingCube = false;
	private boolean intakingCube = false;
	private boolean armsOpen = false;
	private boolean cubeDetected = false;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new CubeCommands());
	}

	public CubeHandler() {
		super();
		leftIntakeMotor = new WPI_TalonSRX(RobotMap.LEFT_INTAKE_MOTOR);
		rightIntakeMotor = new WPI_TalonSRX(RobotMap.RIGHT_INTAKE_MOTOR);
	}

	public void enable(){
	    stopWheels();
	    openArms();
    }

	public void disable(){
	    stopWheels();
    }

	private void spinWheels(double power, boolean inwards) {
	    if(inwards) {
            leftIntakeMotor.set(power);
            rightIntakeMotor.set(-power);
        } else {
	        leftIntakeMotor.set(-power);
	        rightIntakeMotor.set(power);
        }
	}

	private void stopWheels(){
	    leftIntakeMotor.set(0);
	    rightIntakeMotor.set(0);
    }

    public void toggleOutput(){
        stopIntake();
        if(droppingCube){
            stopOutput();
        } else {
            startOutput();
        }
    }

    public void toggleIntake(){
        stopOutput();
	    if(intakingCube){
	        stopIntake();
        } else {
	        startIntake();
        }
    }

    public void toggleArms(){
	    if(armsOpen){
	        closeArms();
        } else {
	        openArms();
        }
    }

    public boolean cubeDetected(){
	    return cubeDetected;
    }

    public void openArms(){
	    armsOpen = true;
	    //TODO Complete this function
    }

    public void closeArms(){
	    armsOpen = false;
	    //TODO Complete this function
    }

    public void startIntake(){
	    spinWheels(intakePower, true);
    }

    public void stopIntake(){
	    stopWheels();
	    intakingCube = false;
    }

    public void startOutput(){
	    spinWheels(outputPower, false);
	    droppingCube = true;
    }

    public void stopOutput(){
	    stopWheels();
	    droppingCube = false;
    }

    public boolean isIntakeStarted(){
	    return intakingCube;
    }
}
