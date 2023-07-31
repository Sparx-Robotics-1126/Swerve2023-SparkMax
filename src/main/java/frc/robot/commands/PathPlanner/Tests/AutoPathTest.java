package frc.robot.commands.PathPlanner.Tests;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class AutoPathTest extends SequentialCommandGroup {

    // private DriveSubsystem m_driveSubsystem;
    private PathPlannerTrajectory m_newPath;

    public AutoPathTest(DriveSubsystem driveSubsystem){
        // m_driveSubsystem = driveSubsystem;
        m_newPath = PathPlanner.loadPath("New Path", new PathConstraints(.5, .5));
        addCommands(driveSubsystem.followTrajectoryCommand(m_newPath, true));
    }

    // @Override
    // public void initialize(){
    //     m_driveSubsystem.followTrajectoryCommand(m_newPath, true);
    // }
}
