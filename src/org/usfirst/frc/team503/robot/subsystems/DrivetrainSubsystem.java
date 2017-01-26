package org.usfirst.frc.team503.robot.subsystems;


import org.usfirst.frc.team503.robot.Robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The robot's drivetrain, which implements the Superstructure abstract class.
 * The drivetrain has several states and builds on the abstract class by
 * offering additional control methods, including control by path and velocity.
 * 
 * @see Subsystem.java
 */
public class DrivetrainSubsystem extends Subsystem {

    private static DrivetrainSubsystem instance = new DrivetrainSubsystem();

    public static DrivetrainSubsystem getInstance() {
        return instance;
    }
    
    
    private void setMotorOutputs(double leftSpeed, double rightSpeed){
		Robot.bot.getCANTalonObj(0).set(-leftSpeed);
    	Robot.bot.getCANTalonObj(1).set(rightSpeed);
    	Robot.bot.getCANTalonObj(2).set(-leftSpeed);
    	Robot.bot.getCANTalonObj(3).set(rightSpeed);
    }

    private final CANTalon leftMaster_, leftSlave_, rightMaster_, rightSlave_;



    // The constructor instantiates all of the drivetrain components when the
    // robot powers up
    private DrivetrainSubsystem() {
        leftMaster_ = Robot.bot.getCANTalonObj(0);
        leftSlave_ = Robot.bot.getCANTalonObj(2);
        rightMaster_ = Robot.bot.getCANTalonObj(1);
        rightSlave_ = Robot.bot.getCANTalonObj(3);
    }

    public void setBrakeMode(boolean on) {
        leftMaster_.enableBrakeMode(on);
        leftSlave_.enableBrakeMode(on);
        rightMaster_.enableBrakeMode(on);
        rightSlave_.enableBrakeMode(on);
    }

	public void arcadeDrive(double moveValue, double rotateValue) {
        double leftMotorSpeed;
        double rightMotorSpeed;
        
        moveValue = limit(moveValue,-1,1);
        rotateValue = limit(rotateValue,-1,1);

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
        setMotorOutputs(leftMotorSpeed, rightMotorSpeed);
    }
	
    public void tankDrive(double leftValue, double rightValue) {
        leftValue = limit(leftValue,-1,1);
        rightValue = limit(rightValue,-1,1);
        setMotorOutputs(leftValue, rightValue);
    }
    
    public static double limit(double num,double min,double max) {
        if (num > max) {
            num= max;
        }
        else if (num < min) {
            num= min;
        }
        	return num;
    }
    
    public CANTalon getLeftMotor(){
    	return leftMaster_;
    }
    
    public CANTalon getRightMotor(){
    	return rightMaster_;
    }
    
    @Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}
	
