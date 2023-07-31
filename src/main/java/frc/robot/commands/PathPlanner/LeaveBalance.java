package frc.robot.commands.PathPlanner;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.driveCommands.DriveToLevel;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PigeonSubsystem;

public class LeaveBalance extends SequentialCommandGroup {

    public LeaveBalance(DriveSubsystem driveSubsystem){

        PathPlannerTrajectory m_conePath = PathPlanner.loadPath("Cone Score 4",
                new PathConstraints(2, 3));
        PathPlannerTrajectory m_pickupPath;

        m_pickupPath = PathPlanner.loadPath("Leave 4 Level Shoot",
                new PathConstraints(1.4,  2.0));

        driveSubsystem.setTrajPID(
                Constants.PathConstants.kpXdefault, Constants.PathConstants.kiXdefault, Constants.PathConstants.kdXdefault,
                Constants.PathConstants.kpYdefault + 3, Constants.PathConstants.kiYdefault, Constants.PathConstants.kdYdefault,
                1.0, Constants.PathConstants.kiRdefault, 0.2);

        addCommands(
//                new InstantCommand(() -> intakeSubsystem.intakeOn(IntakeConstants.kConePickUp), intakeSubsystem),
//                new ArmToPosition(armSubsystem, armPositions.LVLTRE, true).withTimeout(1.20),
                new FollowPathWithEvents(
                        driveSubsystem.followTrajectoryCommand(m_conePath, true),
                        m_conePath.getMarkers(),
                        Constants.AutoConstants.AUTO_EVENT_MAP),
//                new RunCommand(() -> intakeSubsystem.intakeOn(IntakeConstants.kConeEject), intakeSubsystem).withTimeout(.25),

                new FollowPathWithEvents(
                        driveSubsystem.followTrajectoryCommand(m_pickupPath, false, true),
                        m_pickupPath.getMarkers(),
                        Constants.AutoConstants.AUTO_EVENT_MAP),

                new ParallelCommandGroup(
                        new DriveToLevel(driveSubsystem),
//                                .alongWith(new RunCommand(() -> LED.cycle())),
                        new SequentialCommandGroup(
                                new WaitCommand(0.5)
//                                new RunCommand(() -> groundIntake.groundIntakePickUp(), groundIntake).withTimeout(0.5),
//                                new RunCommand(() -> groundIntake.groundIntakeShoot(), groundIntake)
                        )
                )
        );
    }
}
