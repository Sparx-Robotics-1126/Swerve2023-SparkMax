package frc.robot.commands.PathPlanner.Tests;

// import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
// import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem.armPositions;

public class Debug extends SequentialCommandGroup {

//    public Debug(DriveSubsystem driveSubsystem,
//                 ArmSubsystem armSubsystem,
//                 IntakeSubsystem intakeSubsystem,
//                 LEDs LED
//                        ){
 public Debug(DriveSubsystem driveSubsystem,
                LEDSubsystem LED
        ){
        // PathPlannerTrajectory m_firstPath = PathPlanner.loadPath("Cone Score 1", 
        //     new PathConstraints(2, 3));
        //     PathPlannerTrajectory m_secondPath = PathPlanner.loadPath("Cone Reverse 1", 
        //     new PathConstraints(2, 1));
        PathPlannerTrajectory m_thirdPath = PathPlanner.loadPath("Isolation Test", 
            new PathConstraints(0.5, 0.5));
        // PathPlannerTrajectory m_forthPath = PathPlanner.loadPath("Drive Over Cable", 
        //     new PathConstraints(1, 2));
        // PathPlannerTrajectory m_fifthPath = PathPlanner.loadPath("New Drive to Cube 9", 
        //     new PathConstraints(2, 3));
        // PathPlannerTrajectory m_sixthPath = PathPlanner.loadPath("Go to 8 with Cube", 
        //     new PathConstraints(1, 2));

        
        addCommands(
            // new InstantCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kIntakePickUp), intakeSubsystem),
            // new ArmToPositionWithEnd(armSubsystem, armPositions.LVLTRE).withTimeout(1.6),
            // driveSubsystem.followTrajectoryCommand(m_firstPath, true),
            // new RunCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kIntakeReverse), intakeSubsystem).withTimeout(.25),
            // // new WaitCommand(0.5),
            // driveSubsystem.followTrajectoryCommand(m_secondPath, false),
            // new InstantCommand(() -> intakeSubsystem.intakeOff()),
            new FollowPathWithEvents(
                driveSubsystem.followTrajectoryCommand(m_thirdPath, true),
                m_thirdPath.getMarkers(),
                Constants.AutoConstants.AUTO_EVENT_MAP)
//                new RunCommand(() -> intakeSubsystem.intakeOn(IntakeConstants.kConePickUp), intakeSubsystem).alongWith(new RunCommand(() -> LED.cycle())) // TODDO: improve intake constant names
            // new ParallelCommandGroup(new ArmToPositionWithEnd(armSubsystem, armPositions.HOME).withTimeout(2.0),
            //     driveSubsystem.followTrajectoryCommand(m_thirdPath, true))
            // new DriveToLevel(driveSubsystem)
            );
    }
}
