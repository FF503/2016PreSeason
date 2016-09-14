package org.usfirst.frc.team503.robot.subsystems;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CyberShooterSubsystem extends Subsystem implements PIDSource, PIDOutput{
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private static Talon shooterMotor;
	private static Talon pressureMotor;
	private static AnalogInput pressureSensor;
	
	//Configure PID, add tolerance
	public PIDController pressurePID = new PIDController(RobotMap.Cyber.PRESSURE_P, RobotMap.Cyber.PRESSURE_I, RobotMap.Cyber.PRESSURE_D, RobotMap.Cyber.PRESSURE_F, this, this);

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public CyberShooterSubsystem(){
    	shooterMotor = Robot.bot.getTalonObj(0);	//change to adjust to different robots
    	pressureMotor = Robot.bot.getTalonObj(1);
    	pressureSensor = Robot.bot.getAnalogObj(0);
    }
    
    public static CyberShooterSubsystem instance = new CyberShooterSubsystem();
    
    public static void shoot(){ 		//maybe change later
    	shooterMotor.set(1);
    	Timer.delay(1);
    	shooterMotor.set(0);
    }
    
    public static void stopShooter(){
    	shooterMotor.set(0);
    }
    
    public static double getPressure(){		//change to make it accurate
    	return pressureSensor.getVoltage();
    }
    
    public static void setPressureMotor(double power){
    	pressureMotor.set(power);
    }
    
    public static enum Pressure{
    	LOW(1), MID(2), HIGH(3);
    	public double pressure;
    	
    	private Pressure(double pressure){
    		this.pressure = pressure;
    	}
    }

	@Override
	public void pidWrite(double output) {
		RobotMap.Cyber.pressurePIDOutput = output;
	}
	
	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		return getPressure();
	}
	

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return null;
	}    
}

