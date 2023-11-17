// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.LimelightResults;

public class DriveSubsystem extends SubsystemBase {

  // Pigeon gyro
  private final Pigeon2 m_pigeon = new Pigeon2(Constants.PIGEON_ID);

  // Swerve Modules
  private final SwerveModule m_frontLeftModule = new SwerveModule(
      Constants.FRONT_LEFT_DRIVE_MOTOR,
      Constants.FRONT_LEFT_STEER_MOTOR,
      Constants.FRONT_LEFT_STEER_ENCODER,
      Constants.FRONT_LEFT_STEER_OFFSET,
      Constants.TURN_MOTOR_CONFIG,
      Constants.FRONT_LEFT_DRIVE_MOTOR_INVERTED,
      Constants.FRONT_LEFT_TURN_MOTOR_INVERTED);
  private final SwerveModule m_frontRightModule = new SwerveModule(
      Constants.FRONT_RIGHT_DRIVE_MOTOR,
      Constants.FRONT_RIGHT_STEER_MOTOR,
      Constants.FRONT_RIGHT_STEER_ENCODER,
      Constants.FRONT_RIGHT_STEER_OFFSET,
      Constants.TURN_MOTOR_CONFIG,
      Constants.FRONT_RIGHT_DRIVE_MOTOR_INVERTED,
      Constants.FRONT_RIGHT_TURN_MOTOR_INVERTED);
  private final SwerveModule m_backLeftModule = new SwerveModule(
      Constants.BACK_LEFT_DRIVE_MOTOR,
      Constants.BACK_LEFT_STEER_MOTOR,
      Constants.BACK_LEFT_STEER_ENCODER,
      Constants.BACK_LEFT_STEER_OFFSET,
      Constants.TURN_MOTOR_CONFIG,
      Constants.BACK_LEFT_DRIVE_MOTOR_INVERTED,
      Constants.BACK_LEFT_TURN_MOTOR_INVERTED);
  private final SwerveModule m_backRightModule = new SwerveModule(
      Constants.BACK_RIGHT_DRIVE_MOTOR,
      Constants.BACK_RIGHT_STEER_MOTOR,
      Constants.BACK_RIGHT_STEER_ENCODER,
      Constants.BACK_RIGHT_STEER_OFFSET,
      Constants.TURN_MOTOR_CONFIG,
      Constants.BACK_RIGHT_DRIVE_MOTOR_INVERTED,
      Constants.BACK_RIGHT_TURN_MOTOR_INVERTED);

  // Odometry
  private SwerveDrivePoseEstimator m_poseEstimator;

  private double xSpeeds;
  private double ySpeeds;
  private double radsPerSecs;

  public DriveSubsystem() {
    m_pigeon.setYaw(0.0);
    m_poseEstimator = new SwerveDrivePoseEstimator(Constants.DRIVE_KINEMATICS,
      getRotation2d(),
      getModulePositions(),
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
      xSpeeds = 0;
      ySpeeds = 0;
      radsPerSecs = 0;
    }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = Constants.DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_pigeon.getYaw().getValue()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_VELOCITY_METERS_PER_SECOND);
    setModuleStates(swerveModuleStates);
    xSpeeds = xSpeed;
    ySpeeds = ySpeed;
    radsPerSecs = rot;
  }

    public void driveHoloPath(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = Constants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_VELOCITY_METERS_PER_SECOND);
    setModuleStates(swerveModuleStates);
    }

  public void setModuleStates(SwerveModuleState[] states) {
    m_frontLeftModule.setDesiredState(states[0]);
    m_frontRightModule.setDesiredState(states[1]);
    m_backLeftModule.setDesiredState(states[2]);
    m_backRightModule.setDesiredState(states[3]);

    updateOdometry();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw().getValue());
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update(getRotation2d(), getModulePositions());
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds(xSpeeds, ySpeeds, radsPerSecs);
  }

  public void resetOdometry(Pose2d position) {
    m_poseEstimator.resetPosition(getRotation2d(), getModulePositions(), position);
  }

  public void addVision(LimelightResults result) {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      m_poseEstimator.addVisionMeasurement(result.targetingResults.getBotPose2d_wpiBlue(), 
        Timer.getFPGATimestamp());
    } else if (DriverStation.getAlliance().get() == Alliance.Red) {
      m_poseEstimator.addVisionMeasurement(result.targetingResults.getBotPose2d_wpiRed(),
        Timer.getFPGATimestamp());
    }
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(),
        m_backRightModule.getPosition()
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeftModule.getState(),
      m_frontRightModule.getState(),
      m_backLeftModule.getState(),
      m_backRightModule.getState()
    };
  }

  public boolean onChargeStation() {
    return Math.abs(m_pigeon.getPitch().getValue()) > 20;
  }

  public boolean offPitchDown(){
    return Math.abs(m_pigeon.getPitch().getValue()) > 14;
  }

  public boolean onPitchDown() {
    // double[] angleRates = new double[3];
    // m_pigeon.getRawGyro(angleRates);
    // return onChargeStation() && angleRates[0] < -1;
    return Math.abs(m_pigeon.getPitch().getValue()) < 12;
  }

  public boolean onFlat(){
    return Math.abs(m_pigeon.getPitch().getValue()) < 5;
  }

  // public Rotation2d getYaw(){
  //   return Rotation2d.fromDegrees(360 - m_pigeon.getYaw());
  // }

  public void stop() {
    drive(0.0, 0.0, 0.0, false);
  }

  @Override
  public void periodic() {
    if (Math.abs(m_pigeon.getPitch().getValue()) > 50 || Math.abs(m_pigeon.getRoll().getValue()) > 50) {
      Logger.recordOutput("tipping", true);
      stop();
    } else {
      Logger.recordOutput("tipping", false);
    }

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pitch", m_pigeon.getPitch().getValue());
    SmartDashboard.putBoolean("On Charge Station", onChargeStation());
    SmartDashboard.putBoolean("On Pitch Down", onPitchDown());

    Logger.recordOutput("heading", getPose().getRotation().getDegrees() + 180);

    Logger.recordOutput("odometry", getPose());
    // Logger.getInstance().recordOutput("states", getModuleStates());

    Logger.recordOutput("3d pose", new Pose3d(getPose()));

    SwerveModuleState[] measuredStates = new SwerveModuleState[] {null, null, null, null};

    for (int i = 0; i < 4; i++) {
      measuredStates[i] = 
        new SwerveModuleState(
          getModuleStates()[i].speedMetersPerSecond,
          getModuleStates()[i].angle);
    }

    Logger.recordOutput("measured states", measuredStates);
  }

  public Command followTrajectoryCommand(String pathname) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathname);

    return new FollowPathWithEvents(
      new FollowPathHolonomic(
        path,
        this::getPose,
        this::getChassisSpeeds,
        this::driveHoloPath,
        new HolonomicPathFollowerConfig(
          new PIDConstants(Constants.AUTO_X_KP, Constants.AUTO_X_KI, Constants.AUTO_X_KD),
          new PIDConstants(Constants.AUTO_THETA_KP, Constants.AUTO_THETA_KI, Constants.AUTO_THETA_KD),
          Constants.MAX_VELOCITY_METERS_PER_SECOND,
          0.4,
          new ReplanningConfig()
        ),
        this
      ),
      path,
      this::getPose
    );
  }

}
