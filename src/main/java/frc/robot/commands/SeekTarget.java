// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.FiducialNotDetectedException;
import frc.robot.subsystems.vision.Limelight;

/**
 * Turn to and move towards a given target, keeping a certain distance away.
 *
 * <p>This implementation uses areas to approximate distance, so nonstationary targets are
 * permitted.
 */
public class SeekTarget extends Command {

  private static final double TARGET_AREA_CONTROLLER_P = 0.001;
  private static final double TARGET_AREA_CONTROLLER_I = 0.0;
  private static final double TARGET_AREA_CONTROLLER_D = 0.0;

  private static final double TARGET_AREA_ERROR_TOLERANCE = 0.02;
  private static final double LINEAR_SPEED_ERROR_TOLERANCE = 0.001;

  private static final double LINEAR_MAX_SPEED_METERS_PER_SECOND = 0.1;
  private static final double LINEAR_MAX_ACCELERATION_METERS_PER_SECOND = 0.1;

  private static final double TARGET_ANGLE_CONTROLLER_P = 0.025;
  private static final double TARGET_ANGLE_CONTROLLER_I = 0.0;
  private static final double TARGET_ANGLE_CONTROLLER_D = 0.0;

  private static final double ANGLE_ERROR_TOLERANCE_DEGREES = 1;
  private static final double ANGULAR_SPEED_ERROR_TOLERANCE_DEGREES_PER_SECOND = 1;

  private static final double ANGULAR_MAX_SPEED_DEGREES_PER_SECOND = 30;
  private static final double ANGULAR_MAX_ACCELERATION_DEGREES_PER_SECOND = 50;

  private Drivetrain drivetrain;
  private Limelight camera;

  private int fiducialId;

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
    targetAngleController.setTolerance(ANGLE_ERROR_TOLERANCE_DEGREES, ANGULAR_SPEED_ERROR_TOLERANCE_DEGREES_PER_SECOND);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Limelight.Result currentResult;
    try {
      currentResult = camera.getLatestResult();
    } catch (IOException e) {
      e.printStackTrace();
      drivetrain.stop();
      // TODO: For debugging purposes only, please remove later
      SmartDashboard.putNumber("Correcting forward movement", 0);
      SmartDashboard.putNumber("Correcting turning movement", 0);
      return;
    }

    Limelight.Result.Fiducial fiducial;
    try {
      fiducial = currentResult.getFiducialWithId(fiducialId);
    } catch (FiducialNotDetectedException e) {
      drivetrain.stop();
      // TODO: For debugging purposes only, please remove later
      SmartDashboard.putNumber("Correcting forward movement", 0);
      SmartDashboard.putNumber("Correcting turning movement", 0);
      return;
    }

    double targetArea = fiducial.getAreaOfImageConsumed();
    double targetAngle = -Units.degreesToRadians(fiducial.getAngleInCameraView().getZ());

    double correctingForwardMovement, correctingTurningMovement;
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

    // TODO: For debugging purposes only, please remove later
    SmartDashboard.putNumber("Correcting forward movement", correctingForwardMovement);
    SmartDashboard.putNumber("Correcting turning movement", correctingTurningMovement);
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
