// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.unmanaged.Unmanaged;
import com.ctre.phoenixpro.configs.Pigeon2Configuration;
// import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenixpro.hardware.Pigeon2;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.utils.ModuleMap;
import frc.robot.utils.ShuffleboardContent;

public class DriveSubsystem extends SubsystemBase {

  public SwerveDriveKinematics kSwerveKinematics = DriveConstants.kSwerveKinematics;

  public final HashMap<ModulePosition, SwerveModuleSparkMax> m_swerveModules = new HashMap<>(

      Map.of(

          ModulePosition.FRONT_LEFT,
          new SwerveModuleSparkMax(ModulePosition.FRONT_LEFT,
              CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
              CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
              CanConstants.FRONT_LEFT_MODULE_STEER_CANCODER,
              DriveConstants.kFrontLeftDriveMotorReversed,
              DriveConstants.kFrontLeftTurningMotorReversed,
              CanConstants.FRONT_LEFT_MODULE_STEER_OFFSET),

          ModulePosition.FRONT_RIGHT,
          new SwerveModuleSparkMax(
              ModulePosition.FRONT_RIGHT,
              CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
              CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
              CanConstants.FRONT_RIGHT_MODULE_STEER_CANCODER,
              DriveConstants.kFrontRightDriveMotorReversed,
              DriveConstants.kFrontRightTurningMotorReversed,
              CanConstants.FRONT_RIGHT_MODULE_STEER_OFFSET),

          ModulePosition.BACK_LEFT,
          new SwerveModuleSparkMax(ModulePosition.BACK_LEFT,
              CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
              CanConstants.BACK_LEFT_MODULE_STEER_MOTOR,
              CanConstants.BACK_LEFT_MODULE_STEER_CANCODER,
              DriveConstants.kBackLeftDriveMotorReversed,
              DriveConstants.kBackLeftTurningMotorReversed,
              CanConstants.BACK_LEFT_MODULE_STEER_OFFSET),

          ModulePosition.BACK_RIGHT,
          new SwerveModuleSparkMax(
              ModulePosition.BACK_RIGHT,
              CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
              CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
              CanConstants.BACK_RIGHT_MODULE_STEER_CANCODER,
              DriveConstants.kBackRightDriveMotorReversed,
              DriveConstants.kBackRightTurningMotorReversed,
              CanConstants.BACK_RIGHT_MODULE_STEER_OFFSET)));
  // The gyro sensor

  private final PigeonSubsystem m_gyro = PigeonSubsystem.getInstance();

  private PIDController m_xController = new PIDController(DriveConstants.kP_X, 0, DriveConstants.kD_X);
  private PIDController m_yController = new PIDController(DriveConstants.kP_Y, 0, DriveConstants.kD_Y);

  private final PIDController m_pitchPidController = new PIDController(0.0055, 0.00008, 0.0007);

  private ProfiledPIDController m_turnController = new ProfiledPIDController(
      DriveConstants.kP_Theta, 0,
      DriveConstants.kD_Theta,
      Constants.TrapezoidConstants.kThetaControllerConstraints);

  private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
    kSwerveKinematics,
  getHeadingRotation2d(),
  new SwerveModulePosition[]{
    m_swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
    m_swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
    m_swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
    m_swerveModules.get(ModulePosition.BACK_RIGHT).getPosition(),
  },
      new Pose2d(),
      VecBuilder.fill(0.05,0.05,Units.degreesToRadians(5)),
      VecBuilder.fill(0.5,0.5,Units.degreesToRadians(30)));

  private boolean showOnShuffleboard = true;

  // followTrajectoryCommand PID variables
  private double m_kpX;
  private double m_kiX;
  private double m_kdX;

  private double m_kpY;
  private double m_kiY;
  private double m_kdY;

  private double m_kpR;
  private double m_kiR;
  private double m_kdR;
  private int count = 0;
  private SimDouble m_simAngle;// navx sim

  private double m_simYaw;

  public double throttleValue;

  public double targetAngle;

   public boolean m_fieldOriented;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

//    initPigeon();

    resetModuleEncoders();

    setIdleMode(true);

    m_fieldOriented=true;

    if (RobotBase.isSimulation()) {

      var dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

      m_simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")));
    }

    ShuffleboardContent.initMisc(this);
  }

//  /**
//   * Method to drive the robot using joystick info.
//   *
//   * @param xSpeed        Speed of the robot in the x direction (forward).
//   * @param ySpeed        Speed of the robot in the y direction (sideways).
//   * @param rot           Angular rate of the robot.
//   * @param fieldRelative Whether the provided x and y speeds are relative to the
//   *                      field.
//   */
  @SuppressWarnings("ParameterName")

  // move the robot from gamepad
  public void drive(
      double throttle,
      double strafe,
      double rotation,
      
      boolean isOpenLoop) {
    throttle *= DriveConstants.kMaxSpeedMetersPerSecond;
    strafe *= DriveConstants.kMaxSpeedMetersPerSecond;
    rotation *= DriveConstants.kMaxRotationRadiansPerSecond;
    SmartDashboard.putNumber("Rotn1", rotation);
    ChassisSpeeds chassisSpeeds =m_fieldOriented
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            throttle, strafe, rotation, getHeadingRotation2d())
        : new ChassisSpeeds(throttle, strafe, rotation);

    Map<ModulePosition, SwerveModuleState> moduleStates = ModuleMap
        .of(kSwerveKinematics.toSwerveModuleStates(chassisSpeeds));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), DriveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModuleSparkMax module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(moduleStates.get(module.getModulePosition()), isOpenLoop);
  }


  public void resetPigeon(){
    m_gyro.reset();
  }

//  private void initPigeon() {
//    // Factory default the Pigeon.
//    var toApply = new Pigeon2Configuration();
//    // var mountPose = toApply.MountPose;
//    toApply.MountPose.MountPosePitch = 0;
//    // toApply.MountPose.MountPoseRoll = 0;
//    toApply.MountPose.MountPoseYaw = -90;
//    /*
//     * User can change the configs if they want, or leave it empty for
//     * factory-default
//     */
//
////     m_gyro.reset();
//
////     m_gyro.getConfigurator().apply(toApply);
//    // Add runtime adjustments to Pigeon configuration below this line.
//    // _wpiPigeon.configMountPose(AxisDirection.PositiveY, AxisDirection.PositiveZ);
//
//    // used by pro still thinking about it
//    // _pigeon2.getConfigurator().apply(toApply);
//
//    /* Speed up signals to an appropriate rate */
//    // _pigeon2.getYaw().setUpdateFrequency(100);
//    // _pigeon2.getPitch().setUpdateFrequency(100);
//    // _pigeon.reset();
//
//    m_gyro.getPitch().setUpdateFrequency(1000);
//    m_gyro.setYaw(0, .1);
//
//    m_gyro.getYaw().setUpdateFrequency(100);
//    m_gyro.getYaw().waitForUpdate(.1);
//
//    // _pigeon2.setStatusFramePeriod(0,100 )
//    // _pigeon2.getGravityVectorZ().setUpdateFrequency(100);
//}

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry();
    SmartDashboard.putNumber("Yaw",-m_gyro.getYaw());

  }

  public void updateOdometry() {
    m_odometry.update(
        getHeadingRotation2d(),  new SwerveModulePosition[]{
          m_swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
          m_swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
          m_swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
          m_swerveModules.get(ModulePosition.BACK_RIGHT).getPosition(),
        });

    for (SwerveModuleSparkMax module : ModuleMap.orderedValuesList(m_swerveModules)) {
      Translation2d modulePositionFromChassis = DriveConstants.kModuleTranslations
          .get(module.getModulePosition())
          .rotateBy(getHeadingRotation2d())
          .plus(getPoseMeters().getTranslation());
      module.setModulePose(
          new Pose2d(
              modulePositionFromChassis,
              module.getHeadingRotation2d().plus(getHeadingRotation2d())));
    }
  }

  public Pose2d getPoseMeters() {
    return m_odometry.getEstimatedPosition();
  }

  public SwerveDrivePoseEstimator getOdometry() {
    return m_odometry;
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
            Rotation2d.fromDegrees(-m_gyro.getAngle()),
            new SwerveModulePosition[] {
                    m_swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
                    m_swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
                    m_swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
                    m_swerveModules.get(ModulePosition.BACK_RIGHT).getPosition()
            },
            pose);
  }

  public void setOdometry(Pose2d pose) {

    m_odometry.resetPosition(pose.getRotation(),  new SwerveModulePosition[]{
      m_swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
      m_swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
      m_swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
      m_swerveModules.get(ModulePosition.BACK_RIGHT).getPosition(),
    }, pose);
    m_gyro.reset();

  }

  public SwerveModuleSparkMax getSwerveModule(ModulePosition modulePosition) {
    return m_swerveModules.get(modulePosition);
  }

  public double getHeadingDegrees() {
    return -Math.IEEEremainder((m_gyro.getAngle()), 360);

  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public Map<ModulePosition, SwerveModuleState> getModuleStates() {
    Map<ModulePosition, SwerveModuleState> map = new HashMap<>();
    for (ModulePosition i : m_swerveModules.keySet()) {
      map.put(i, m_swerveModules.get(i).getState());
    }
    return map;
  }

//  /**
//   * Sets the swerve ModuleStates.
//   *
//   * @param desiredStates The desired SwerveModule states.
//   */
  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModuleSparkMax module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(states[module.getModulePosition().ordinal()], isOpenLoop);
  }
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    setSwerveModuleStates(desiredStates,false);
//    SwerveDriveKinematics.desaturateWheelSpeeds(
//            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
//
//    m_swerveModules.get(ModulePosition.FRONT_LEFT).setDesiredState(desiredStates[0]);
//    m_swerveModules.get(ModulePosition.FRONT_RIGHT).setDesiredState(desiredStates[1]);
//    m_swerveModules.get(ModulePosition.BACK_LEFT).setDesiredState(desiredStates[2]);
//    m_swerveModules.get(ModulePosition.BACK_RIGHT).setDesiredState(desiredStates[3]);
  }
  public void resetModuleEncoders() {
    for (SwerveModuleSparkMax module : ModuleMap.orderedValuesList(m_swerveModules))
      module.resetAngleToAbsolute();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    // m_gyro.reset();
    // m_gyro.setAngleAdjustment(0);

  }

  public Translation2d getTranslation() {
    return getPoseMeters().getTranslation();
  }

  public PIDController getXPidController() {
    return m_xController;
  }

  public PIDController getYPidController() {
    return m_yController;
  }

  public void setSwerveModuleStatesAuto(SwerveModuleState[] states) {
    setSwerveModuleStates(states, false);
  }

  public ProfiledPIDController getThetaPidController() {
    return m_turnController;
  }

  public double getX() {
    return getTranslation().getX();
  }

  public double getY() {
    return getTranslation().getY();
  }

  public double reduceRes(double value, int numPlaces) {
    double n = Math.pow(10, numPlaces);
    return Math.round(value * n) / n;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // public double getTurnRate() {
  // return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  // }

  public void setIdleMode(boolean brake) {
    for (SwerveModuleSparkMax module : ModuleMap.orderedValuesList(m_swerveModules)) {
      module.setDriveBrakeMode(brake);
      module.setTurnBrakeMode(brake);
    }

  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeedSim = kSwerveKinematics.toChassisSpeeds(
        ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));
    // want to simulate navX gyro changing as robot turns
    // information available is radians per second and this happens every 20ms
    // radians/2pi = 360 degrees so 1 degree per second is radians / 2pi
    // increment is made every 20 ms so radian adder would be (rads/sec) *(20/1000)
    // degree adder would be radian adder * 360/2pi
    // so degree increment multiplier is 360/100pi = 1.1459

    double temp = chassisSpeedSim.omegaRadiansPerSecond * 1.1459155;

    temp += m_simAngle.get();

    m_simAngle.set(temp);

    Unmanaged.feedEnable(20);
  }

  public void turnModule(ModulePosition mp, double speed) {
    getSwerveModule(mp).turnMotorMove(speed);
  }

  public void positionTurnModule(ModulePosition mp, double angle) {
    getSwerveModule(mp).positionTurn(angle);
  }

  public void driveModule(ModulePosition mp, double speed) {
    getSwerveModule(mp).driveMotorMove(speed);
  }

  public boolean getTurnInPosition(ModulePosition mp, double targetAngle) {
    return getSwerveModule(mp).turnInPosition(targetAngle);
  }

  public double getAnglefromThrottle() {

    return 180 * throttleValue;
  }

  public void setX() {
    m_swerveModules.get(ModulePosition.FRONT_LEFT).setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_swerveModules.get(ModulePosition.FRONT_RIGHT).setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_swerveModules.get(ModulePosition.BACK_LEFT).setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_swerveModules.get(ModulePosition.BACK_RIGHT).setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public Command DriveCommand(double speed){
    return new StartEndCommand(
      () -> drive( speed,0,0,false), 
      () -> drive( 0,0,0,false),
      this);
  }
  //Charge Station Autos
  public void driveToLevel(){
    double pidOut = MathUtil.clamp(m_pitchPidController.calculate(
            m_gyro.getRoll(), 0), -DriveConstants.kAutoLevelMaxOutput, DriveConstants.kAutoLevelMaxOutput);
    drive( pidOut, 0, 0, false);

    if (++count %10 == 0) {
      System.out.println("Roll is :" + m_gyro.getRoll());
      System.out.println("Pitch is :" + m_gyro.getPitch());
      System.out.println("PID Output is: " + pidOut);
    }

    SmartDashboard.putNumber("DrivePidOutput",pidOut);
  }

  public boolean isLevel(){
    return Math.abs(m_gyro.getRoll()) < Constants.AutoConstants.kLevelTolerance;
  }

  public boolean driveToAngle (double targetAngle){
    boolean atAngle = true;
    double currentAngle = m_gyro.getRoll();
    // double currentAngle = m_odometry.;
    if ( currentAngle >= targetAngle) {
      drive( .25, 0, 0, false);  // xSpeed .4
      atAngle = false;
      if (++count %10 == 0) {
        System.out.println("Angle is:" + currentAngle);
      }
    }
    else {
      drive( 0, 0, 0, true);
    }
    return atAngle;
  }


  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> {
              // Reset odometry for the first path you run during auto
              if(isFirstPath){
                PathPlannerTrajectory transformed = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());
                this.resetOdometry(transformed.getInitialHolonomicPose());
              }
            }),
            new PPSwerveControllerCommand(
                    traj,
                    this::getPoseMeters, // Pose supplier
                    Constants.DriveConstants.kSwerveKinematics, // SwerveDriveKinematics
                    new PIDController(Constants.PathConstants.kpXdefault, PathConstants.kiXdefault, PathConstants.kdXdefault), // Forward/Backward X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                    new PIDController(PathConstants.kpYdefault, PathConstants.kiYdefault, PathConstants.kdYdefault), // Strafe Y controller (usually the same values as X controller)
                    new PIDController(PathConstants.kpRdefault, PathConstants.kiRdefault, PathConstants.kdRdefault), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                    this::setModuleStates, // Module states consumer
                    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                    this // Requires this drive subsystem

                    //TODO: 1 inch undershot Forward/Backward. Increasing Xkp and Xki increases this error
            )
    );
  }
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath, boolean isModifiedPID) {
    if (!isModifiedPID){
      m_kpX = PathConstants.kpXdefault;
      m_kiX = PathConstants.kiXdefault;
      m_kdX = PathConstants.kdXdefault;

      m_kpY = PathConstants.kpYdefault;
      m_kiY = PathConstants.kiYdefault;
      m_kdY = PathConstants.kdYdefault;

      m_kpR = PathConstants.kpRdefault;
      m_kiR = PathConstants.kiRdefault;
      m_kdR = PathConstants.kdRdefault;
    }
    return new SequentialCommandGroup(
            new InstantCommand(() -> {
              // Reset odometry for the first path you run during auto
              if(isFirstPath){
                PathPlannerTrajectory transformed = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());
                this.resetOdometry(transformed.getInitialHolonomicPose());
              }
            }),
            new PPSwerveControllerCommand(
                    traj,
                    this::getPoseMeters, // Pose supplier
                    Constants.DriveConstants.kSwerveKinematics, // SwerveDriveKinematics
                    new PIDController(m_kpX, m_kiX, m_kdX), // Forward/Backward X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                    new PIDController(m_kpY, m_kiY, m_kdY), // Strafe Y controller (usually the same values as X controller)
                    new PIDController(m_kpR, m_kiR, m_kdR), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                    this::setModuleStates, // Module states consumer
                    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                    this // Requires this drive subsystem
            )
    );
  }


  public void setTrajPID(
          double kpX, double kiX, double kdX,
          double kpY, double kiY, double kdY,
          double kpR, double kiR, double kdR
  ){
    m_kpX = kpX;  m_kpY = kpY;  m_kpR = kpR;
    m_kiX = kiX;  m_kiY = kiY;  m_kiR = kiR;
    m_kdX = kdX;  m_kdY = kdY;  m_kdR = kdR;
  }

  public void setXTrajPID(double kp, double ki, double kd){
    m_kpX = kp;
    m_kiX = ki;
    m_kdX = kd;
  }
  public void setYTrajPID(double kp, double ki, double kd){
    m_kpY = kp;
    m_kiY = ki;
    m_kdY = kd;
  }
  public void setRotTrajPID(double kp, double ki, double kd){
    m_kpR = kp;
    m_kiR = ki;
    m_kdR = kd;
  }
}
