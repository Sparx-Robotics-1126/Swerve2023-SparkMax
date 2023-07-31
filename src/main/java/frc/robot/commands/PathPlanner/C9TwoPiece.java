//package frc.robot.commands.PathPlanner;
//
//import com.pathplanner.lib.PathConstraints;
//import com.pathplanner.lib.PathPlanner;
//import com.pathplanner.lib.PathPlannerTrajectory;
//import com.pathplanner.lib.commands.FollowPathWithEvents;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.Constants;
//import frc.robot.Constants.PathConstants;
//import frc.robot.commands.armCommands.ArmToPosition;
//import frc.robot.commands.driveCommands.DriveToLevel;
//import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
//import frc.robot.subsystems.LEDs;
//import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem;
//import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem.armPositions;
//import frc.robot.subsystems.MainIntakeSubsystem.IntakeSubsystem;
//
//public class C9TwoPiece extends SequentialCommandGroup {
//
//    public C9TwoPiece(
//        int endStrategy,
//        boolean isRED,
//        DriveSubsystem driveSubsystem,
//        ArmSubsystem armSubsystem,
//        IntakeSubsystem intakeSubsystem,
//        LEDs LED
//        ){
//
//    PathPlannerTrajectory m_conePath = PathPlanner.loadPath("Cone Score 9",
//        new PathConstraints(2, 3));
//    PathPlannerTrajectory m_PickupPath = PathPlanner.loadPath("Full Drive to Cube 9",
//        new PathConstraints(2, 1.6));
//    PathPlannerTrajectory m_lastPath;
//
//    SequentialCommandGroup m_lastCommands;
//
//    if (endStrategy == PathConstants.LVL){
//        if (isRED)
//            m_lastPath = PathPlanner.loadPath("Balance from 8 RED",
//                new PathConstraints(3.0, 3.0));
//        else
//            m_lastPath = PathPlanner.loadPath("Balance from 8 RED",     // Add Red Option if Charge Station behaves differently
//                new PathConstraints(3.0, 3.0));
//
//        m_lastCommands = new SequentialCommandGroup(
//            new FollowPathWithEvents(
//                driveSubsystem.followTrajectoryCommand(m_lastPath, false),
//                m_lastPath.getMarkers(),
//                Constants.AutoConstants.AUTO_EVENT_MAP),
//            new DriveToLevel(driveSubsystem)
//                .alongWith(new RunCommand(() -> LED.cycle())));
//    } else {
//        // if (lastCommands == PathConstants.NoLVL){                                        // Default initiallization required
//            m_lastPath = PathPlanner.loadPath("Cube Reverse 8 (from 9)",
//                new PathConstraints(2, 2));
//
//            m_lastCommands = new SequentialCommandGroup(
//                driveSubsystem.followTrajectoryCommand(m_lastPath, false),
//                new ArmToPosition(armSubsystem, armPositions.HOME).withTimeout(2.0),
//                new RunCommand(() -> LED.cycle(), LED));
//
//        // }
//    }
//
//    addCommands(
//        new InstantCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kConePickUp), intakeSubsystem),
//        new ArmToPosition(armSubsystem, armPositions.LVLTRE, true).withTimeout(1.20),
//        new FollowPathWithEvents(
//            driveSubsystem.followTrajectoryCommand(m_conePath, true),
//            m_conePath.getMarkers(),
//            Constants.AutoConstants.AUTO_EVENT_MAP),
//        new RunCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kConeEject), intakeSubsystem).withTimeout(.25),
//
//        new FollowPathWithEvents(
//            driveSubsystem.followTrajectoryCommand(m_PickupPath, false),
//            m_PickupPath.getMarkers(),
//            Constants.AutoConstants.AUTO_EVENT_MAP),
//        new InstantCommand(() -> LED.setColor(LED.BLUE)),
//
//        m_lastCommands
//
//        // new FollowPathWithEvents(
//        //     driveSubsystem.followTrajectoryCommand(m_lastPath, false),
//        //     m_lastPath.getMarkers(),
//        //     Constants.AutoConstants.AUTO_EVENT_MAP),
//        // new DriveToLevel(driveSubsystem)
//        //     .alongWith(new RunCommand(() -> LED.cycle()))
//        );
//    }
//}
