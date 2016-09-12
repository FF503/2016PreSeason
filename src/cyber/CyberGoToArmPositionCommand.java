package cyber;

import org.usfirst.frc.team503.robot.RobotMap;
import org.usfirst.frc.team503.robot.subsystems.CyberArmSubsystem;
import org.usfirst.frc.team503.robot.subsystems.CyberArmSubsystem.ArmAngle;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CyberGoToArmPositionCommand extends Command {
	double angle;
    public CyberGoToArmPositionCommand(ArmAngle angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.angle = angle.angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	RobotMap.Cyber.currentAngle = CyberArmSubsystem.getArmAngle();
    	CyberArmSubsystem.instance.armPID.enable();
    	CyberArmSubsystem.instance.armPID.setSetpoint(angle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(RobotMap.Cyber.currentAngle<(angle-RobotMap.Cyber.ARM_TOLERANCE)){
    		CyberArmSubsystem.setArmPower(RobotMap.Cyber.armPIDOutput);
    	}
    	else if(RobotMap.Cyber.currentAngle>(angle+RobotMap.Cyber.ARM_TOLERANCE)){
    		CyberArmSubsystem.setArmPower(-RobotMap.Cyber.armPIDOutput);
    	}
    	else{
    		CyberArmSubsystem.setArmPower(0);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return CyberArmSubsystem.instance.armPID.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	CyberArmSubsystem.setArmPower(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
