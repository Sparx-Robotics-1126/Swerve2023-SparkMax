// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
// import frc.robot.commands.auto.DriveForward;
// import frc.robot.commands.auto.FiveBallAuto;
import frc.robot.commands.LED.CANdleConfigCommands;
import frc.robot.commands.LED.CANdlePrintCommands;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PigeonSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final PigeonSubsystem m_pigeon;
  final DriveSubsystem m_robotDrive = new DriveSubsystem();

  public final FieldSim m_fieldSim = new FieldSim(m_robotDrive);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  // The driver's controller
  private final XboxController m_driveController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  private final LEDSubsystem m_candleSubsystem = new LEDSubsystem(m_operatorController);
  static Joystick leftJoystick = new Joystick(OIConstants.kDriverControllerPort);

//  private XboxController m_coDriverController = new XboxController(OIConstants.kOperatorControllerPort);

//  final GamepadButtons driver = new GamepadButtons(m_coDriverController, true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Preferences.removeAll();
    Pref.deleteUnused();
    Pref.addMissing();
    m_pigeon = PigeonSubsystem.getInstance();
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    // Configure the button bindings

    m_fieldSim.initSim();
    initializeAutoChooser();

    m_robotDrive.setDefaultCommand(
            new SetSwerveDrive(
                    m_robotDrive,
                    () -> m_driveController.getLeftY(),
                    () -> m_driveController.getLeftX(),
                    () -> m_driveController.getRightX()));

    if(m_candleSubsystem != null) {
      configureOperatorButtonBindings();
    }
//        m_robotDrive.setDefaultCommand(
//        new SetSwerveDrive(
//            m_robotDrive,
//            () -> leftJoystick.getRawAxis(XboxController.Axis.kLeftY.value),
//            () -> leftJoystick.getRawAxis(XboxController.Axis.kLeftX.value),
//            () -> leftJoystick.getRawAxis(XboxController.Axis.kRightX.value)));

  }


  private void configureOperatorButtonBindings() {
    new JoystickButton(m_operatorController, Constants.LEDConstants.BlockButton).whenPressed(m_candleSubsystem::setColors, m_candleSubsystem);
    new JoystickButton(m_operatorController, Constants.LEDConstants.IncrementAnimButton).whenPressed(m_candleSubsystem::incrementAnimation, m_candleSubsystem);
    new JoystickButton(m_operatorController, Constants.LEDConstants.DecrementAnimButton).whenPressed(m_candleSubsystem::decrementAnimation, m_candleSubsystem);

    new POVButton(m_operatorController, Constants.LEDConstants.MaxBrightnessAngle).whenPressed(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 1.0));
    new POVButton(m_operatorController, Constants.LEDConstants.MidBrightnessAngle).whenPressed(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 0.3));
    new POVButton(m_operatorController, Constants.LEDConstants.ZeroBrightnessAngle).whenPressed(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 0));

    new JoystickButton(m_operatorController, Constants.LEDConstants.VbatButton).whenPressed(new CANdlePrintCommands.PrintVBat(m_candleSubsystem));
    new JoystickButton(m_operatorController, Constants.LEDConstants.V5Button).whenPressed(new CANdlePrintCommands.Print5V(m_candleSubsystem));
    new JoystickButton(m_operatorController, Constants.LEDConstants.CurrentButton).whenPressed(new CANdlePrintCommands.PrintCurrent(m_candleSubsystem));
    new JoystickButton(m_operatorController, Constants.LEDConstants.TemperatureButton).whenPressed(new CANdlePrintCommands.PrintTemperature(m_candleSubsystem));
  }


  private void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
    // m_autoChooser.addOption("Drive Forward", new DriveForward(m_robotDrive));
    // m_autoChooser.addOption("5 Ball Auto", new FiveBallAuto(m_robotDrive));

    SmartDashboard.putData("Auto Selector", m_autoChooser);

  }

  public void resetPigeon(){
    m_robotDrive.resetPigeon();
  }

  public void simulationPeriodic() {
    m_fieldSim.periodic();
    periodic();
  }

  public void periodic() {
    m_fieldSim.periodic();
  }

  public double getThrottle() {
    return -leftJoystick.getThrottle();
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }

}
