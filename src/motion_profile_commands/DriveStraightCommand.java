package motion_profile_commands;

import org.usfirst.frc.team503.robot.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import util.Rotation2d;

/**
 *
 */
public class DriveStraightCommand extends Command {
	private double heading;
	private double velocity;
	private double distance;
	private double startDistance;
	private DrivetrainSubsystem drive = DrivetrainSubsystem.getInstance(); 
	
    public DriveStraightCommand(double distance, double velocity, double heading) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.heading = heading;
    	this.distance = distance;
    	this.velocity = velocity;
    }
    
    public DriveStraightCommand(double distance, double velocity){
    	this(distance, velocity, 0);
    }
    
    // Called just before this Command runs the first time
    protected void initialize() {
    	startDistance = drive.getCurrentDistance();
    	drive.setVelocityHeadingSetpoint(velocity, Rotation2d.fromDegrees(heading));
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	drive.updateVelocityHeadingSetpoint();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (drive.getCurrentDistance() - startDistance) >= distance ;
    }

    // Called once after isFinished returns true
    protected void end() {
    	drive.setVelocitySetpoint(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}