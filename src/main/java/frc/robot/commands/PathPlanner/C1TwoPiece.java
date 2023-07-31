//package frc.robot.commands.PathPlanner;
//
//import com.pathplanner.lib.PathConstraints;
//import com.pathplanner.lib.PathPlanner;
//import com.pathplanner.lib.PathPlannerTrajectory;
//import com.pathplanner.lib.commands.FollowPathWithEvents;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.IntakeConstants;
//import frc.robot.Constants.PathConstants;
//import frc.robot.commands.armCommands.ArmToPosition;
//import frc.robot.commands.armCommands.ArmToPositionWithEnd;
//import frc.robot.commands.driveCommands.DriveToLevel;
//import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
//import frc.robot.subsystems.LEDs;
//import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem;
//import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem.armPositions;
//import frc.robot.subsystems.MainIntakeSubsystem.IntakeSubsystem;
//
//public class C1TwoPiece extends SequentialCommandGroup {
//
//    public C1TwoPiece(
//        int endStrategy,
//        boolean isRED,
//        DriveSubsystem driveSubsystem,
//        ArmSubsystem armSubsystem,
//        IntakeSubsystem intakeSubsystem,
//        LEDs LED
//        ){
//
//    PathPlannerTrajectory m_conePath = PathPlanner.loadPath("Cone Score 1",
//        new PathConstraints(2, 3));
//    PathPlannerTrajectory m_pickupPath;
//    PathPlannerTrajectory m_lastPath;
//
//    SequentialCommandGroup m_lastCommands;
//
//    // Cube Pick-Up
//    if (isRED) {
//            m_pickupPath = PathPlanner.loadPath("Cube to 2 from 1 Grid RED",
//                new PathConstraints(2, 2.2));
//    } else {
//            m_pickupPath = PathPlanner.loadPath("Cube to 2 from 1 Grid BLUE",
//                new PathConstraints(2, 2.2));
//    }
//
//    // Balance
//    if (endStrategy == PathConstants.LVL){
//        if (isRED) {
//            m_lastPath = PathPlanner.loadPath("Balance from 2 RED",
//                new PathConstraints(2.5, 2.5));
//        } else {
//            m_lastPath = PathPlanner.loadPath("Balance from 2 BLUE",
//                new PathConstraints(2.5, 2.5));
//        }
//
//        m_lastCommands = new SequentialCommandGroup(
//            new FollowPathWithEvents(
//                driveSubsystem.followTrajectoryCommand(m_lastPath, false),
//                m_lastPath.getMarkers(),
//                AutoConstants.AUTO_EVENT_MAP),
//            new DriveToLevel(driveSubsystem)
//                .alongWith(new RunCommand(() -> LED.cycle())));
//
//    // No Balance
//    } else {
//        // if (lastCommands == PathConstants.NoLVL){                                        // Default initiallization required
//            m_lastPath = PathPlanner.loadPath("Cube Reverse 2 DRIVEOUT",
//                new PathConstraints(2, 2));
//
//            m_lastCommands = new SequentialCommandGroup(
//                driveSubsystem.followTrajectoryCommand(m_lastPath, false),
//                new ArmToPositionWithEnd(armSubsystem, armPositions.HOME).withTimeout(2.0),
//                new RunCommand(() -> LED.cycle(), LED));
//
//        // // Grab Cube, No Balance
//        // } else {                                                                         // Not tested / dialed-in
//        //     if (lastCommands == PathConstants.grabCube){
//        //         m_lastPath = PathPlanner.loadPath("Cube 2 Pickup from 2",
//        //             new PathConstraints(2.0, 2.2));
//
//        //         m_lastCommands = new SequentialCommandGroup(
//        //             new FollowPathWithEvents(
//        //                 driveSubsystem.followTrajectoryCommand(m_lastPath, false),
//        //                 m_lastPath.getMarkers(),
//        //                 AutoConstants.AUTO_EVENT_MAP)
//        //         );
//        //     }
//        // }
//    }
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
//            driveSubsystem.followTrajectoryCommand(m_pickupPath, false),
//            m_pickupPath.getMarkers(),
//            AutoConstants.AUTO_EVENT_MAP),
//        new RunCommand(() -> intakeSubsystem.intakeOn(IntakeConstants.kCubeEject), intakeSubsystem).withTimeout(0.25)   // TODO: Possible to Reduce?
//            .alongWith(new InstantCommand(() -> LED.setColor(LED.BLUE))),
//
//        m_lastCommands
//
//        // new FollowPathWithEvents(
//        //     driveSubsystem.followTrajectoryCommand(m_lastPath, false),
//        //     m_lastPath.getMarkers(),
//        //     AutoConstants.AUTO_EVENT_MAP),
//        // new DriveToLevel(driveSubsystem)
//        //     .alongWith(new RunCommand(() -> LED.cycle()))
//        );
//    }
//}
