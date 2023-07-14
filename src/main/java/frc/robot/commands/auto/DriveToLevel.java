package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class DriveToLevel extends CommandBase {
    private DriveSubsystem m_drivetrain;
    private Timer timer = new Timer(); 

    public DriveToLevel(DriveSubsystem drivetrain){
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    @Override 
    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        m_drivetrain.driveToLevel();
    }

    @Override
    public boolean isFinished(){
        boolean isLevel = m_drivetrain.isLevel(); 
        double currentTime = timer.get();
        if (isLevel){

            if (currentTime >= 2)
                return true; 
            
        } else
            timer.reset();  //Only reset timer if not level
        return false;
    }
}
