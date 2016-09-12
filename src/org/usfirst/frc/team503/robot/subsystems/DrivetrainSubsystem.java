package org.usfirst.frc.team503.robot.subsystems;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DrivetrainSubsystem extends Subsystem {

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public static DrivetrainSubsystem instance = new DrivetrainSubsystem();
    
    private void setMotorOutputs(double leftSpeed, double rightSpeed, boolean scaledInputs){
    	if(scaledInputs==true){
    		Robot.bot.getCANTalonObj(0).set(scaleInput(-leftSpeed));
    		Robot.bot.getCANTalonObj(1).set(scaleInput(rightSpeed));
    		Robot.bot.getCANTalonObj(2).set(scaleInput(-leftSpeed));
    		Robot.bot.getCANTalonObj(3).set(scaleInput(rightSpeed));
    	}
    	else{
    		Robot.bot.getCANTalonObj(0).set(-leftSpeed);
	    	Robot.bot.getCANTalonObj(1).set(rightSpeed);
	    	Robot.bot.getCANTalonObj(2).set(-leftSpeed);
	    	Robot.bot.getCANTalonObj(3).set(rightSpeed);
    	}
    }
    
    private static double limit(double num) {
        if (num > 1.0) {
            num= 1.0;
        }
        else if (num < -1.0) {
            num= -1.0;
        }
        	return num;
    }
    
	private double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.0, 0.03, 0.06, 0.09, 0.13, 0.17, 0.21,
				0.26, 0.31, 0.36, 0.41, 0.47, 0.53, 0.61, .80, 1.00 };
		
		// get the corresponding index for the scaleInput array.
		boolean neg = false;
		if(dVal<0){
			neg = true;
		}
		/*	//joystick position function
		dVal = Math.abs(dVal);
		int index = (int) (dVal * 16.0);
		if (index > 16){
			index = 16;
		}
		
		double dScale = 0.0;
		if (neg==true) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}*/
		
		//time function
		double dScale = 0;
		if(dVal>RobotMap.Cyber.driveValue || dVal == 1){
			RobotMap.Cyber.driveCounter++;
			RobotMap.Cyber.driveValue = dVal;
		}
		else if(dVal<RobotMap.Cyber.driveValue || dVal == -1){
			RobotMap.Cyber.driveCounter--;
			RobotMap.Cyber.driveValue = dVal;
		}
		else{
			RobotMap.Cyber.driveValue = dVal;
		}
		if(neg==true){
			dScale = -scaleArray[RobotMap.Cyber.driveCounter];
		}
		else if(neg == false){
			dScale = scaleArray[RobotMap.Cyber.driveCounter];
		}
		return dScale;
	}
    
	public void arcadeDrive(double moveValue, double rotateValue, boolean scaledInputs) {
        double leftMotorSpeed;
        double rightMotorSpeed;
        
        moveValue = limit(moveValue);
        rotateValue = limit(rotateValue);

        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            } else {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }
        }
        setMotorOutputs(leftMotorSpeed, rightMotorSpeed, scaledInputs);
    }
	
    public void tankDrive(double leftValue, double rightValue, boolean scaledInputs) {
        leftValue = limit(leftValue);
        rightValue = limit(rightValue);
        setMotorOutputs(leftValue, rightValue, scaledInputs);
    }
}

