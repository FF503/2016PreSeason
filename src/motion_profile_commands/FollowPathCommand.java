package motion_profile_commands;

import org.usfirst.frc.team503.robot.subsystems.DrivetrainControlSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import util.Path;

/**
 *
 */
public class FollowPathCommand extends Command {
	private Path path;
	private boolean reverse;
	private boolean hasStarted;
	private DrivetrainControlSubsystem drive = DrivetrainControlSubsystem.getInstance();
	
    public FollowPathCommand(Path path, boolean reverse) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.path = path;
    	this.reverse = reverse;
    	hasStarted = false;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	hasStarted=true;
    	drive.followPath(path, reverse);    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	drive.updatePathFollower();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return drive.isFinishedPath() && hasStarted;
    }

    // Called once after isFinished returns true
    protected void end() {
    	drive.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	drive.stop();
    }
}
