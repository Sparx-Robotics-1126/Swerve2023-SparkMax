package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToAngle extends CommandBase {
    private DriveSubsystem m_drivetrain;
    private boolean atAngle = false;
    private boolean Initlevel;

    public DriveToAngle(DriveSubsystem drivetrain){
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    public void initialize(){
        atAngle = false;
        Initlevel = m_drivetrain.isLevel();
        SmartDashboard.putBoolean("DriveToAngle InitLevel", Initlevel);
    }

    @Override
    public void execute(){
        atAngle = m_drivetrain.driveToAngle(Constants.AutoConstants.kDriveAngle);
        SmartDashboard.putBoolean("atAngle", atAngle);
    }

    @Override
    public void end(boolean isInterrupted) {
        // m_drivetrain.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished(){
        if (Initlevel) return atAngle;
        else return true; 
    }
}
