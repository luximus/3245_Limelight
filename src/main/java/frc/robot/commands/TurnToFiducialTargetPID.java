// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.FiducialNotDetectedException;
import frc.robot.subsystems.vision.Limelight;

public class TurnToFiducialTargetPID extends ProfiledPIDCommand {

  private static final double CONTROLLER_P = 0.0;
  private static final double CONTROLLER_I = 0.0;
  private static final double CONTROLLER_D = 0.0;

  private static final double CONTROLLER_MAX_VELOCITY_DEGREES_PER_SECOND = 45;
  private static final double CONTROLLER_MAX_ACCELERATION_DEGREES_PER_SECOND = 90;

  private static final double ANGLE_TOLERANCE_DEGREES = 1;
  private static final double ANGULAR_VELOCITY_ERROR_TOLERANCE_DEGREES_PER_SECOND = 1;

  private Limelight camera;
  private int targetFiducialId;
  private double angleDegrees = 0.0;

  /** Creates a new TurnToFiducialTargetPID. */
  public TurnToFiducialTargetPID(Drivetrain drivetrain, Limelight camera, int fiducialId) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            CONTROLLER_P,
            CONTROLLER_I,
            CONTROLLER_D,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(CONTROLLER_MAX_VELOCITY_DEGREES_PER_SECOND,
                                             CONTROLLER_MAX_ACCELERATION_DEGREES_PER_SECOND)),
        // This should return the measurement
        () -> 0.0,
        0.0,
        // This uses the output
        (output, setpoint) -> {
          drivetrain.driveArcadeStyle(0, output);
        },
        drivetrain);

    this.m_measurement = this::getError;
    this.camera = camera;
    this.targetFiducialId = fiducialId;

    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(ANGLE_TOLERANCE_DEGREES, ANGULAR_VELOCITY_ERROR_TOLERANCE_DEGREES_PER_SECOND);
  }

  private double getError() {
    Limelight.Result currentResult = null;
    try {
      currentResult = camera.getLatestResult();
    } catch (IOException e) {
      e.printStackTrace();
    }

    if (currentResult != null) {
      try {
        Limelight.Result.Fiducial fiducial = currentResult.getFiducialWithId(targetFiducialId);
        angleDegrees = -Units.radiansToDegrees(fiducial.getAngleInCameraView().getZ());
      } catch (FiducialNotDetectedException e) {}
    }

    return angleDegrees;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
