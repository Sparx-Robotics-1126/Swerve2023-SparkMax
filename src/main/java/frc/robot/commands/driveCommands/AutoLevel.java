package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;

public class AutoLevel extends SequentialCommandGroup {

    public AutoLevel(DriveSubsystem driveSubsystem) {
        addCommands(
                new InstantCommand(() -> SmartDashboard.putString("AutoLevel", "DriveToAngle")),
                new DriveToAngle(driveSubsystem),
                new InstantCommand(() -> SmartDashboard.putString("AutoLevel", "Drive1.5")),
                driveSubsystem.DriveCommand(0.2).withTimeout(1.5),
                new InstantCommand(() -> SmartDashboard.putString("AutoLevel", "DriveToLevel")),
                new DriveToLevel(driveSubsystem)
        );
    }
}

