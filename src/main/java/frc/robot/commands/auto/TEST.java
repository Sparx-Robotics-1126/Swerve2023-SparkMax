package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TEST extends SequentialCommandGroup {

    public TEST(
        int whichPath,
        DriveSubsystem driveSubsystem
        ){

    PathPlannerTrajectory m_testPath;

    // SequentialCommandGroup m_lastCommands;
    
    // Balance
    if (whichPath == 0){
        m_testPath = PathPlanner.loadPath("PID TEST x & y", 
            new PathConstraints(2.5, 2.5));
        
    } else {
        if (whichPath == 1)
            m_testPath = PathPlanner.loadPath("PID TEST rot", 
                new PathConstraints(2.5, 2.5));     
    else {
        if (whichPath == 2)
            m_testPath = PathPlanner.loadPath("PID TEST all", 
                new PathConstraints(2.5, 2.5));
    else {
    m_testPath = PathPlanner.loadPath("PID TEST straight", 
        new PathConstraints(2.5, 2.5));
    }
    }

    driveSubsystem.setTrajPID(
        PathConstants.kpXdefault, PathConstants.kiXdefault, PathConstants.kdXdefault, 
        PathConstants.kpYdefault*4, PathConstants.kiYdefault, PathConstants.kdYdefault, 
        PathConstants.kpRdefault, PathConstants.kiRdefault, PathConstants.kdRdefault);

    
    addCommands(
        driveSubsystem.followTrajectoryCommand(m_testPath, true, true)
        // new FollowPathWithEvents(
        //     driveSubsystem.followTrajectoryCommand(m_testPath, false),
        //     m_testPath.getMarkers(),
        //     AutoConstants.AUTO_EVENT_MAP)
        );
    }
}
}

