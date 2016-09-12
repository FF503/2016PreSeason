package cyber;

import org.usfirst.frc.team503.robot.RobotMap;
import org.usfirst.frc.team503.robot.subsystems.CyberShooterSubsystem;
import org.usfirst.frc.team503.robot.subsystems.CyberShooterSubsystem.Pressure;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CyberPressurizeCommand extends Command {
	double pressure;
	
    public CyberPressurizeCommand(Pressure pressure) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.pressure = pressure.pressure;
    	requires(CyberShooterSubsystem.instance);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	RobotMap.Cyber.currentPressure = CyberShooterSubsystem.getPressure();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(RobotMap.Cyber.currentPressure<(pressure-RobotMap.Cyber.PRESSURE_TOLERANCE)){
    		CyberShooterSubsystem.setPressureMotor(RobotMap.Cyber.PRESSURE_MOTOR_SPEED);
    	}
    	else if(RobotMap.Cyber.currentPressure>(pressure+RobotMap.Cyber.PRESSURE_TOLERANCE)){
    		CyberShooterSubsystem.setPressureMotor(-RobotMap.Cyber.PRESSURE_MOTOR_SPEED);
    		
    	}
    	else{
    		CyberShooterSubsystem.setPressureMotor(0);
    	}
    	RobotMap.Cyber.currentPressure = CyberShooterSubsystem.getPressure();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return ((RobotMap.Cyber.currentPressure>(pressure-RobotMap.Cyber.PRESSURE_TOLERANCE)) && (RobotMap.Cyber.currentPressure<(CyberShooterSubsystem.getPressure()+RobotMap.Cyber.PRESSURE_TOLERANCE)));
    }

    // Called once after isFinished returns true
    protected void end() {
    	CyberShooterSubsystem.setPressureMotor(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
