// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class MAXSwerveModule {

  private static final double kFreeSpeedRpm = 5676;

  // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
  // This changes the drive speed of the module (a pinion gear with more teeth will result in a
  // robot that drives faster).
  private static final int kDrivingMotorPinionTeeth = 14;

  // Invert the turning encoder, since the output shaft rotates in the opposite direction of
  // the steering motor in the MAXSwerve Module.
  private static final boolean kTurningEncoderInverted = true;

  // Calculations required for driving motor conversion factors and feed forward
  private static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60;
  private static final double kWheelDiameterMeters = 0.0762;
  private static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
  // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
  private static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
  private static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
    / kDrivingMotorReduction;

  private static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
    / kDrivingMotorReduction; // meters
  private static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
    / kDrivingMotorReduction) / 60.0; // meters per second

  private static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
  private static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

  private static final double kTurningEncoderPositionPIDMinInput = 0; // radians
  private static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

  private static final double kDrivingP = 0.04;
  private static final double kDrivingI = 0;
  private static final double kDrivingD = 0;
  private static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
  private static final double kDrivingMinOutput = -1;
  private static final double kDrivingMaxOutput = 1;

  private static final double kTurningP = 1;
  private static final double kTurningI = 0;
  private static final double kTurningD = 0;
  private static final double kTurningFF = 0;
  private static final double kTurningMinOutput = -1;
  private static final double kTurningMaxOutput = 1;

  private static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
  private static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

  private static final int kDrivingMotorCurrentLimit = 50; // amps
  private static final int kTurningMotorCurrentLimit = 20; // amps

  private final CANSparkMax drivingSparkMax;
  private final CANSparkMax turningSparkMax;

  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController drivingPIDController;
  private final SparkMaxPIDController turningPIDController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    drivingSparkMax.restoreFactoryDefaults();
    turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    drivingEncoder = drivingSparkMax.getEncoder();
    turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    drivingPIDController = drivingSparkMax.getPIDController();
    turningPIDController = turningSparkMax.getPIDController();
    drivingPIDController.setFeedbackDevice(drivingEncoder);
    turningPIDController.setFeedbackDevice(turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    drivingEncoder.setPositionConversionFactor(kDrivingEncoderPositionFactor);
    drivingEncoder.setVelocityConversionFactor(kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turningEncoder.setPositionConversionFactor(kTurningEncoderPositionFactor);
    turningEncoder.setVelocityConversionFactor(kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    turningEncoder.setInverted(kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningPIDController.setPositionPIDWrappingMinInput(kTurningEncoderPositionPIDMinInput);
    turningPIDController.setPositionPIDWrappingMaxInput(kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    drivingPIDController.setP(kDrivingP);
    drivingPIDController.setI(kDrivingI);
    drivingPIDController.setD(kDrivingD);
    drivingPIDController.setFF(kDrivingFF);
    drivingPIDController.setOutputRange(kDrivingMinOutput,
        kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    turningPIDController.setP(kTurningP);
    turningPIDController.setI(kTurningI);
    turningPIDController.setD(kTurningD);
    turningPIDController.setFF(kTurningFF);
    turningPIDController.setOutputRange(kTurningMinOutput,
        kTurningMaxOutput);

    drivingSparkMax.setIdleMode(kDrivingMotorIdleMode);
    turningSparkMax.setIdleMode(kTurningMotorIdleMode);
    drivingSparkMax.setSmartCurrentLimit(kDrivingMotorCurrentLimit);
    turningSparkMax.setSmartCurrentLimit(kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    drivingSparkMax.burnFlash();
    turningSparkMax.burnFlash();

    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(drivingEncoder.getVelocity(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        drivingEncoder.getPosition(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    this.desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }
}