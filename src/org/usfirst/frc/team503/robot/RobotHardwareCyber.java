package org.usfirst.frc.team503.robot;

import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

public class RobotHardwareCyber extends RobotHardware {
	
	private static CANTalon frontLeftMotor;
	private static CANTalon frontRightMotor; 
	private static CANTalon backLeftMotor;
	private static CANTalon backRightMotor;
	private static Spark leftArmMotor;
	private static Spark rightArmMotor;
	private static Talon shooterMotor;
	private static Talon pressureMotor;
	private static DigitalInput armUpperLimitSwitch;
	private static DigitalInput armLowerLimitSwitch;
	private static AnalogInput pressureSensor;
	private static AnalogAccelerometer accelerometer;
	
	@Override
	public void initialize(){
		frontLeftMotor = new CANTalon(0); // practice bot values, change for cyber
		frontRightMotor = new CANTalon(2);
		backLeftMotor = new CANTalon(3);
		backRightMotor = new CANTalon(1);
		leftArmMotor = new Spark(1);
		rightArmMotor = new Spark(3);
		shooterMotor = new Talon(0);
		pressureMotor = new Talon(2);
		//armUpperLimitSwitch = new DigitalInput(0);
		//armLowerLimitSwitch = new DigitalInput(1);
		pressureSensor = new AnalogInput(0);
		//accelerometer = new AnalogAccelerometer(1);
	}
	
	@Override
	public CANTalon getCANTalonObj(int CANTalonID){
		if(CANTalonID == 0){
			return frontLeftMotor;
		}
		else if(CANTalonID == 1){
			return frontRightMotor;
		}
		else if(CANTalonID == 2){
			return backLeftMotor;
		}
		else if(CANTalonID == 3){
			return backRightMotor;
		}
		else{
			return null;
		}
	}
	
	@Override
	public Talon getTalonObj(int TalonID){
		if(TalonID == 0){
			return shooterMotor;
		}
		else if(TalonID == 1){
			return pressureMotor;
		}
		else{
			return null;
		}
	}
	
	@Override
	public Spark getSparkObj(int SparkID){
		if(SparkID == 0){
			return leftArmMotor;
		}
		else if(SparkID == 1){
			return rightArmMotor;
		}
		else{
			return null;
		}
	}
	
	@Override
	public DigitalInput getDigitalObj (int DigitalID){
		if(DigitalID == 0){
			return armUpperLimitSwitch;
		}
		else if(DigitalID == 1){
			return armLowerLimitSwitch;
		}
		else{
			return null;
		}
	}
	
	@Override
	public AnalogInput getAnalogObj(int AnalogID){
		if(AnalogID == 0){
			return pressureSensor;
		}
		else{
			return null;
		}
	}
	
	@Override
	public AnalogAccelerometer getAccelObj(int AccelID){
		if(AccelID==0){
			return accelerometer;
		}
		else{
			return null;
		}
	}
	
	@Override
	public String getName(){
		return "Cyber";
	}
}