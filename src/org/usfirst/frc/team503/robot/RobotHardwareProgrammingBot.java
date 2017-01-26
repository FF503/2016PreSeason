package org.usfirst.frc.team503.robot;

import com.ctre.CANTalon;

public class RobotHardwareProgrammingBot extends RobotHardware {
	
	private static CANTalon frontLeftMotor;
	private static CANTalon frontRightMotor; 
	private static CANTalon backLeftMotor;
	private static CANTalon backRightMotor;
	
	
	
	@Override
	public void initialize(){
		frontLeftMotor = new CANTalon(1); // practice bot values, change for cyber
		frontRightMotor = new CANTalon(3);
		backLeftMotor = new CANTalon(2);
		backRightMotor = new CANTalon(4);
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
	public String getName(){
		return "ProgrammingBot";
	}
	
}