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
//import frc.robot.commands.armCommands.ArmToPosition;
//import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
//import frc.robot.subsystems.LEDs;
//import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem;
//import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem.armPositions;
//import frc.robot.subsystems.MainIntakeSubsystem.IntakeSubsystem;
//
//public class C9OneConeShootTwo extends SequentialCommandGroup {
//
//    public C9OneConeShootTwo(DriveSubsystem driveSubsystem,
//                        ArmSubsystem armSubsystem,
//                        IntakeSubsystem intakeSubsystem,
//                        LEDs LED
//                        ){
//
//        PathPlannerTrajectory m_firstPath = PathPlanner.loadPath("Cone Score 9",
//            new PathConstraints(2, 3));
//        PathPlannerTrajectory m_secondPath = PathPlanner.loadPath("Full Drive to Shoot 9",
//            new PathConstraints(4, 1.65));
//        // PathPlannerTrajectory m_thirdPath = PathPlanner.loadPath("Balance from 8 RED",
//        //     new PathConstraints(3.0, 3.0));
//
//        addCommands(
//            new InstantCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kConePickUp), intakeSubsystem),
//            new ArmToPosition(armSubsystem, armPositions.LVLTRE, true).withTimeout(1.40),
//            new FollowPathWithEvents(
//                driveSubsystem.followTrajectoryCommand(m_firstPath, true),
//                m_firstPath.getMarkers(),
//                Constants.AutoConstants.AUTO_EVENT_MAP),
//            new RunCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kConeEject), intakeSubsystem).withTimeout(.25),
//            new FollowPathWithEvents(
//                driveSubsystem.followTrajectoryCommand(m_secondPath, false),
//                m_secondPath.getMarkers(),
//                Constants.AutoConstants.AUTO_EVENT_MAP)
//            // new InstantCommand(() -> LED.setColor(LED.BLUE)),
//            // new FollowPathWithEvents(
//            //     driveSubsystem.followTrajectoryCommand(m_thirdPath, false),
//            //     m_thirdPath.getMarkers(),
//            //     Constants.AutoConstants.AUTO_EVENT_MAP),
//            // new DriveToLevel(driveSubsystem)
//            //     .alongWith(new RunCommand(() -> LED.cycle()))
//            );
//    }
//}
