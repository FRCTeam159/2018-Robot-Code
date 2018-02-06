package org.usfirst.frc.team159.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import org.usfirst.frc.team159.robot.RobotMap;
import org.usfirst.frc.team159.robot.commands.CubeCommands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CubeHandler extends Subsystem {

	private WPI_TalonSRX leftIntakeMotor;
	private WPI_TalonSRX rightIntakeMotor;
//	private Ultrasonic cubeDetector;
//	private DoubleSolenoid armPneumatic;

	private static final double intakePower = 0.5;
	private static final double outputPower = 0.25;
	private static final double detectionRange = 6;

	private boolean outputStarted = false;
	private boolean intakeStarted = false;
	private boolean armsOpen = false;

	public void initDefaultCommand() {
		// Set the default command for this subsystem here.
		setDefaultCommand(new CubeCommands());
	}

	public CubeHandler() {
		super();
		leftIntakeMotor = new WPI_TalonSRX(RobotMap.LEFT_INTAKE_MOTOR);
		rightIntakeMotor = new WPI_TalonSRX(RobotMap.RIGHT_INTAKE_MOTOR);

		// commented out because it throws exceptions if the IDs are wrong
//		cubeDetector = new Ultrasonic(RobotMap.CUBE_DETECTOR_PING_CHANNEL, RobotMap.CUBE_DETECTOR_ECHO_CHANNEL);
//		cubeDetector.setAutomaticMode(true);
//		armPneumatic = new DoubleSolenoid(RobotMap.ARM_PISTON_ID, RobotMap.SOLENOID_FORWARD, RobotMap.SOLENOID_REVERSE);
	}

	public void enable(){
	    stopWheels();
	    openArms();
    }

	public void disable(){
	    stopWheels();
	    leftIntakeMotor.disable();
	    rightIntakeMotor.disable();
    }

	// Put methods for controlling this subsystem here. Call these from Commands.

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

    public void stop(){
        stopWheels();
        outputStarted = false;
        intakeStarted = false;
    }

    public void toggleIntake(){
        stop();
	    if(intakeStarted){
	        stop();
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
	    // TODO test
//	    return cubeDetector.getRangeInches() < detectionRange;
    	return false;
    }

    private void openArms(){
//	    armPneumatic.set(DoubleSolenoid.Value.kReverse);
	    armsOpen = true;
    }

    private void closeArms(){
//      armPneumatic.set(DoubleSolenoid.Value.kForward);
	    armsOpen = false;
    }

    private void startIntake(){
	    spinWheels(intakePower, true);
	    outputStarted = false;
	    intakeStarted = true;
    }

    public void startOutput(){
	    spinWheels(outputPower, false);
	    outputStarted = true;
	    intakeStarted = false;
    }


    public boolean isIntakeStarted(){
	    return intakeStarted;
    }

    public boolean isOutputStarted(){
		return outputStarted;
	}
}
