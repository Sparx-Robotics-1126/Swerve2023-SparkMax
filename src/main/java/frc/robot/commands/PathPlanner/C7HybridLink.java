//package frc.robot.commands.PathPlanner;
//
//import com.pathplanner.lib.PathConstraints;
//import com.pathplanner.lib.PathPlanner;
//import com.pathplanner.lib.PathPlannerTrajectory;
//import com.pathplanner.lib.commands.FollowPathWithEvents;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.Constants;
//import frc.robot.commands.armCommands.ArmToPosition;
//import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
//import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem;
//import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem.armPositions;
//import frc.robot.subsystems.MainIntakeSubsystem.IntakeSubsystem;
//
//public class C7HybridLink extends SequentialCommandGroup {
//
//    public C7HybridLink(DriveSubsystem driveSubsystem,
//                        ArmSubsystem armSubsystem,
//                        IntakeSubsystem intakeSubsystem
//                        ){
//
//        PathPlannerTrajectory m_firstPath = PathPlanner.loadPath("Full Drive to Shoot 7",
//            new PathConstraints(4, 1.65));
//
//
//        addCommands(
//            new InstantCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kConeEject), intakeSubsystem),
//            new ArmToPosition(armSubsystem, armPositions.LVLONE, true).withTimeout(0.45),
//            new FollowPathWithEvents(
//                driveSubsystem.followTrajectoryCommand(m_firstPath, true),
//                m_firstPath.getMarkers(),
//                Constants.AutoConstants.AUTO_EVENT_MAP)
//            );
//    }
//}
