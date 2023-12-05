// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.SwerveUtils;

public class Drivetrain extends SubsystemBase {

  // Driving Parameters - Note that these are not the maximum capable speeds of
  // the robot, rather the allowed maximum speeds
  private static final double kMaxSpeedMetersPerSecond = 4.8;
  private static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

  private static final double kDirectionSlewRate = 1.2; // radians per second
  private static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
  private static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

  // Chassis configuration
  private static final double kTrackWidth = Units.inchesToMeters(26.5);
  // Distance between centers of right and left wheels on robot
  private static final double kWheelBase = Units.inchesToMeters(26.5);
  // Distance between front and back wheels on robot
  private static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    new Translation2d(kWheelBase / 2, kTrackWidth / 2),
    new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
    new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

  // Angular offsets of the modules relative to the chassis in radians
  private static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
  private static final double kFrontRightChassisAngularOffset = 0;
  private static final double kBackLeftChassisAngularOffset = Math.PI;
  private static final double kBackRightChassisAngularOffset = Math.PI / 2;

  // SPARK MAX CAN IDs
  private static final int kFrontLeftDrivingCanId = 11;
  private static final int kRearLeftDrivingCanId = 13;
  private static final int kFrontRightDrivingCanId = 15;
  private static final int kRearRightDrivingCanId = 17;

  private static final int kFrontLeftTurningCanId = 10;
  private static final int kRearLeftTurningCanId = 12;
  private static final int kFrontRightTurningCanId = 14;
  private static final int kRearRightTurningCanId = 16;

  private static final boolean kGyroReversed = false;

  private static final IdleMode kDefaultIdleMode = IdleMode.kBrake;

  // Create MAXSwerveModules
  private final MAXSwerveModule frontLeftModule = new MAXSwerveModule(
      kFrontLeftDrivingCanId,
      kFrontLeftTurningCanId,
      kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule frontRightModule = new MAXSwerveModule(
      kFrontRightDrivingCanId,
      kFrontRightTurningCanId,
      kFrontRightChassisAngularOffset);

  private final MAXSwerveModule rearLeftModule = new MAXSwerveModule(
      kRearLeftDrivingCanId,
      kRearLeftTurningCanId,
      kBackLeftChassisAngularOffset);

  private final MAXSwerveModule rearRightModule = new MAXSwerveModule(
      kRearRightDrivingCanId,
      kRearRightTurningCanId,
      kBackRightChassisAngularOffset);

  // The gyro sensor
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(kMagnitudeSlewRate);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private IdleMode idleMode = kDefaultIdleMode;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      kDriveKinematics,
      Rotation2d.fromDegrees(gyro.getAngle()),
      new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          rearLeftModule.getPosition(),
          rearRightModule.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
    frontLeftModule.setIdleMode(idleMode);
    frontRightModule.setIdleMode(idleMode);
    rearLeftModule.setIdleMode(idleMode);
    rearRightModule.setIdleMode(idleMode);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(kDirectionSlewRate / currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }


      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.getAngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        currentTranslationDir = SwerveUtils.stepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        }
        else {
          currentTranslationDir = SwerveUtils.wrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        currentTranslationDir = SwerveUtils.stepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentRotation = rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * kMaxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * kMaxAngularSpeed;

    var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, kMaxSpeedMetersPerSecond);
    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    rearLeftModule.setDesiredState(swerveModuleStates[2]);
    rearRightModule.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setToXFormation() {
    frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, kMaxSpeedMetersPerSecond);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    rearLeftModule.setDesiredState(desiredStates[2]);
    rearRightModule.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    rearLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    rearRightModule.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Get the idle mode for all swerve modules in this drivetrain. If the idle mode of an individual swerve module was
   * modified, this will not reflect that change.
   */
  public IdleMode getIdleMode() {
    return idleMode;
  }

  /**
   * Set the idle mode for all swerve modules in this drivetrain.
   */
  public void setIdleMode(IdleMode idleMode) {
    this.idleMode = idleMode;
    frontLeftModule.setIdleMode(idleMode);
    frontRightModule.setIdleMode(idleMode);
    rearLeftModule.setIdleMode(idleMode);
    rearRightModule.setIdleMode(idleMode);
  }

  public static SwerveDriveKinematics getKinematics() {
    return kDriveKinematics;
  }
}