//package frc.robot.commands.PathPlanner;
//
//import com.pathplanner.lib.PathConstraints;
//import com.pathplanner.lib.PathPlanner;
//import com.pathplanner.lib.PathPlannerTrajectory;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.Constants;
//import frc.robot.commands.armCommands.ArmToPositionWithEnd;
//import frc.robot.commands.driveCommands.DriveToLevel;
//import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
//import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem;
//import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem.armPositions;
//import frc.robot.subsystems.MainIntakeSubsystem.IntakeSubsystem;
//
//public class C4C6OneConeLevel extends SequentialCommandGroup {
//
//    public C4C6OneConeLevel(DriveSubsystem driveSubsystem,
//                        ArmSubsystem armSubsystem,
//                        IntakeSubsystem intakeSubsystem
//                        ){
//
//        PathPlannerTrajectory m_firstPath = PathPlanner.loadPath("Cone Score 6",
//            new PathConstraints(2, 3));
//            PathPlannerTrajectory m_secondPath = PathPlanner.loadPath("Cone Reverse 6",
//            new PathConstraints(2, 1));
//        PathPlannerTrajectory m_thirdPath = PathPlanner.loadPath("Leave 6 Level",
//            new PathConstraints(1.4, 3));
//
//
//        addCommands(
//            new InstantCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kConePickUp), intakeSubsystem),
//            new ArmToPositionWithEnd(armSubsystem, armPositions.LVLTRE).withTimeout(1.55),
//            driveSubsystem.followTrajectoryCommand(m_firstPath, true),
//            new RunCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kConeEject), intakeSubsystem).withTimeout(.25),
//            driveSubsystem.followTrajectoryCommand(m_secondPath, false),
//            new InstantCommand(() -> intakeSubsystem.intakeOff()),
//            new ParallelCommandGroup(new ArmToPositionWithEnd(armSubsystem, armPositions.HOME).withTimeout(1.25)),
//            driveSubsystem.followTrajectoryCommand(m_thirdPath, false),
//            new DriveToLevel(driveSubsystem)
//            );
//    }
//
//    // @Override
//    // public void initialize(){
//    //     m_driveSubsystem.followTrajectoryCommand(m_newPath, true);
//    // }
//}
