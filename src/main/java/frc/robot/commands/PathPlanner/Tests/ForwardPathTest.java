package frc.robot.commands.PathPlanner.Tests;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class ForwardPathTest extends SequentialCommandGroup {

 
    private PathPlannerTrajectory m_newPath;

    public ForwardPathTest(DriveSubsystem driveSubsystem){
        
        m_newPath = PathPlanner.loadPath("ForwardPathTest", new PathConstraints(.5, .5));
        addCommands(driveSubsystem.followTrajectoryCommand(m_newPath, true));
    }

}
