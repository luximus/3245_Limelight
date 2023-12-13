// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.FiducialNotDetectedException;
import frc.robot.subsystems.vision.Limelight;

public class SeekTarget extends CommandBase {

  private static final double TARGET_AREA_CONTROLLER_P = 0.0;
  private static final double TARGET_AREA_CONTROLLER_I = 0.0;
  private static final double TARGET_AREA_CONTROLLER_D = 0.0;

  private static final double TARGET_AREA_ERROR_TOLERANCE = 0.01;
  private static final double LINEAR_SPEED_ERROR_TOLERANCE = 0.1;

  private static final double LINEAR_MAX_SPEED_METERS_PER_SECOND = 3;
  private static final double LINEAR_MAX_ACCELERATION_METERS_PER_SECOND = 6;

  private static final double TARGET_ANGLE_CONTROLLER_P = 0.0;
  private static final double TARGET_ANGLE_CONTROLLER_I = 0.0;
  private static final double TARGET_ANGLE_CONTROLLER_D = 0.0;

  private static final double ANGLE_ERROR_TOLERANCE = 1;
  private static final double ANGULAR_SPEED_ERROR_TOLERANCE = 3;

  private static final double ANGULAR_MAX_SPEED_DEGREES_PER_SECOND = 30;
  private static final double ANGULAR_MAX_ACCELERATION_DEGREES_PER_SECOND = 60;

  private Drivetrain drivetrain;
  private Limelight camera;

  private int fiducialId;

  private double targetArea;
  private double targetAngle;

  private ProfiledPIDController targetAreaController = new ProfiledPIDController(
    TARGET_AREA_CONTROLLER_P,
    TARGET_AREA_CONTROLLER_I,
    TARGET_AREA_CONTROLLER_D,
    new TrapezoidProfile.Constraints(
      LINEAR_MAX_SPEED_METERS_PER_SECOND,
      LINEAR_MAX_ACCELERATION_METERS_PER_SECOND
    ));
  private ProfiledPIDController targetAngleController = new ProfiledPIDController(
    TARGET_ANGLE_CONTROLLER_P,
    TARGET_ANGLE_CONTROLLER_I,
    TARGET_ANGLE_CONTROLLER_D,
    new TrapezoidProfile.Constraints(
      ANGULAR_MAX_SPEED_DEGREES_PER_SECOND,
      ANGULAR_MAX_ACCELERATION_DEGREES_PER_SECOND
    )
  );

  /** Creates a new SeekTarget. */
  public SeekTarget(Drivetrain drivetrain, Limelight camera, int fiducialId, double goalArea) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.camera = camera;

    targetAngleController.enableContinuousInput(-180, 180);

    targetAreaController.setGoal(goalArea);
    targetAngleController.setGoal(0.0);

    targetAreaController.setTolerance(TARGET_AREA_ERROR_TOLERANCE, LINEAR_SPEED_ERROR_TOLERANCE);
    targetAngleController.setTolerance(ANGLE_ERROR_TOLERANCE, ANGULAR_SPEED_ERROR_TOLERANCE);

    targetArea = goalArea;
    targetAngle = 0.0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double correctingForwardMovement, correctingTurningMovement;

    Limelight.Result currentResult = null;
    try {
      currentResult = camera.getLatestResult();
    } catch (IOException e) {
      e.printStackTrace();
    }

    if (currentResult != null) {
      try {
        Limelight.Result.Fiducial fiducial = currentResult.getFiducialWithId(fiducialId);
        targetArea = fiducial.getAreaOfImageConsumed();
        targetAngle = -Units.degreesToRadians(fiducial.getAngleInCameraView().getZ());
      } catch (FiducialNotDetectedException e) {}
    }

    if (!targetAreaController.atGoal()) {
      correctingForwardMovement = targetAreaController.calculate(targetArea);
    } else {
      correctingForwardMovement = 0;
    }
    if (!targetAngleController.atGoal()) {
      correctingTurningMovement = targetAngleController.calculate(targetAngle);
    } else {
      correctingTurningMovement = 0;
    }

    drivetrain.driveArcadeStyle(correctingForwardMovement, correctingTurningMovement);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
