package cyber_commands;

import org.usfirst.frc.team503.robot.OI;
import org.usfirst.frc.team503.robot.subsystems.CyberArmSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CyberTeleopArmCommand extends Command {

    public CyberTeleopArmCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	requires(CyberArmSubsystem.instance);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(CyberArmSubsystem.getArmLowerLimitSwitch() == true){
    		if(OI.getRightYValue()<0){
    			CyberArmSubsystem.setArmPower(0);
    		}
    		else{
    			CyberArmSubsystem.setArmPower(OI.getRightYValue());
    		}
    	}
    	else if(CyberArmSubsystem.getArmUpperLimitSwitch() == true){
    		if(OI.getRightYValue()>0){
    			CyberArmSubsystem.setArmPower(0);
    		}
    		else{
    			CyberArmSubsystem.setArmPower(OI.getRightYValue());
    		}
    	}
    	else{
    		CyberArmSubsystem.setArmPower(OI.getRightYValue());
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return !DriverStation.getInstance().isOperatorControl();
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
