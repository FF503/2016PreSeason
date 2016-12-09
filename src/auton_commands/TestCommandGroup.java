package auton_commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.command.CommandGroup;
import motion_profile_commands.FollowPathCommand;
import util.Path;
import util.Path.Waypoint;
import util.Translation2d;

/**
 *
 */
public class TestCommandGroup extends CommandGroup {
    
    public  TestCommandGroup() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	List<Waypoint> path = new ArrayList<Waypoint>();
    	path.add(new Waypoint(new Translation2d(40,0), 10));
    	addSequential(new FollowPathCommand(new Path(path), false));
    }
}
