//package frc.robot.commands.PathPlanner;
//
//import com.pathplanner.lib.PathConstraints;
//import com.pathplanner.lib.PathPlanner;
//import com.pathplanner.lib.PathPlannerTrajectory;
//import com.pathplanner.lib.commands.FollowPathWithEvents;
//import edu.wpi.first.wpilibj2.command.*;
//import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.IntakeConstants;
//import frc.robot.Constants.PathConstants;
//import frc.robot.commands.armCommands.ArmToPosition;
//import frc.robot.commands.driveCommands.DriveToLevel;
//import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
//import frc.robot.subsystems.GroundIntakeSubsystem.GroundIntake;
//import frc.robot.subsystems.LEDs;
//import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem;
//import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem.armPositions;
//import frc.robot.subsystems.MainIntakeSubsystem.IntakeSubsystem;
//
//public class C4C6TwoPieceCS extends SequentialCommandGroup {
//
//    public C4C6TwoPieceCS(
//        // int endStrategy,
//        boolean isRED,
//        DriveSubsystem driveSubsystem,
//        ArmSubsystem armSubsystem,
//        IntakeSubsystem intakeSubsystem,
//        LEDs LED,
//        GroundIntake groundIntake
//        ){
//
//    PathPlannerTrajectory m_conePath = PathPlanner.loadPath("Cone Score 4",
//        new PathConstraints(2, 3));
//    PathPlannerTrajectory m_pickupPath;
//    // PathPlannerTrajectory m_lastPath;
//
//    // SequentialCommandGroup m_lastCommands;
//
//    // Cube Pick-Up
//    if (isRED) {
//            m_pickupPath = PathPlanner.loadPath("Leave 4 Level Shoot",
//                new PathConstraints(1.4, 2.0));
//    } else {
//            m_pickupPath = PathPlanner.loadPath("Leave 4 Level Shoot",
//                new PathConstraints(1.4,  2.0));
//    }
//
//        // m_lastCommands = new SequentialCommandGroup(
//        //     new FollowPathWithEvents(
//        //         driveSubsystem.followTrajectoryCommand(m_lastPath, false),
//        //         m_lastPath.getMarkers(),
//        //         AutoConstants.AUTO_EVENT_MAP),
//        //     new DriveToLevel(driveSubsystem)
//        //         .alongWith(new RunCommand(() -> LED.cycle())));
//
//        driveSubsystem.setTrajPID(
//            PathConstants.kpXdefault, PathConstants.kiXdefault, PathConstants.kdXdefault,
//            PathConstants.kpYdefault + 3, PathConstants.kiYdefault, PathConstants.kdYdefault,
//            1.0, PathConstants.kiRdefault, 0.2);
//
//
//
//    addCommands(
//        new InstantCommand(() -> intakeSubsystem.intakeOn(IntakeConstants.kConePickUp), intakeSubsystem),
//        new ArmToPosition(armSubsystem, armPositions.LVLTRE, true).withTimeout(1.20),
//        new FollowPathWithEvents(
//            driveSubsystem.followTrajectoryCommand(m_conePath, true),
//            m_conePath.getMarkers(),
//            AutoConstants.AUTO_EVENT_MAP),
//        new RunCommand(() -> intakeSubsystem.intakeOn(IntakeConstants.kConeEject), intakeSubsystem).withTimeout(.25),
//
//        new FollowPathWithEvents(
//            driveSubsystem.followTrajectoryCommand(m_pickupPath, false, true),
//            m_pickupPath.getMarkers(),
//            AutoConstants.AUTO_EVENT_MAP),
//
//        new ParallelCommandGroup(
//            new DriveToLevel(driveSubsystem)
//                .alongWith(new RunCommand(() -> LED.cycle())),
//            new SequentialCommandGroup(
//                new WaitCommand(0.5),
//                new RunCommand(() -> groundIntake.groundIntakePickUp(), groundIntake).withTimeout(0.5),
//                new RunCommand(() -> groundIntake.groundIntakeShoot(), groundIntake)
//            )
//        )
//        );
//    }
//}
