package frc.robot.commands.PathPlanner.Tests;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class ReversePathTest extends SequentialCommandGroup {

 
    private PathPlannerTrajectory m_newPath;

    public ReversePathTest(DriveSubsystem driveSubsystem){
        
        m_newPath = PathPlanner.loadPath("ReversePathTest", new PathConstraints(.5, .5));
        addCommands(driveSubsystem.followTrajectoryCommand(m_newPath, true));
    }

}
