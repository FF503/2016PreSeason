package org.usfirst.frc.team503.robot.subsystems;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CyberArmSubsystem extends Subsystem implements PIDSource, PIDOutput{
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private static Spark leftArmMotor;
	private static Spark rightArmMotor;
	private static DigitalInput armUpperLimitSwitch;
	private static DigitalInput	armLowerLimitSwitch;
	private static AnalogAccelerometer accelerometer;
	
	public PIDController armPID = new PIDController(RobotMap.Cyber.ARM_P, RobotMap.Cyber.ARM_I, RobotMap.Cyber.ARM_D, RobotMap.Cyber.ARM_F, this, this);
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public CyberArmSubsystem(){
    	leftArmMotor = Robot.bot.getSparkObj(0);
    	rightArmMotor = Robot.bot.getSparkObj(1);
    	armUpperLimitSwitch = Robot.bot.getDigitalObj(0);
    	armLowerLimitSwitch = Robot.bot.getDigitalObj(1);
    	accelerometer = Robot.bot.getAccelObj(3);
    }
    
    public static CyberArmSubsystem instance = new CyberArmSubsystem();
    
    public static boolean getArmUpperLimitSwitch(){
    	return armUpperLimitSwitch.get();
    }
    public static boolean getArmLowerLimitSwitch(){
    	return armLowerLimitSwitch.get();
    }
    
    public static double getArmAngle(){		//test to make accurate
    	return accelerometer.getAcceleration();
    }
    
    public static void setArmPower(double power){
    	leftArmMotor.set(power);
    	rightArmMotor.set(power);
    }
    
    public static enum ArmAngle{
    	BOTTOM(0), LOW(30), MID(45), HIGH(60), TOP(90);
    	public double angle;
    
    	private ArmAngle(double angle){
    		this.angle = angle;
    	}
    }

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		RobotMap.Cyber.armPIDOutput = output;
	}
	
	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		return getArmAngle();
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

