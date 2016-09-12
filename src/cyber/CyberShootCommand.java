package cyber;

import org.usfirst.frc.team503.robot.OI;
import org.usfirst.frc.team503.robot.subsystems.CyberShooterSubsystem;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CyberShootCommand extends Command {

    public CyberShootCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(OI.getSafety1() && OI.getSafety2()){
    		CyberShooterSubsystem.shoot();
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
