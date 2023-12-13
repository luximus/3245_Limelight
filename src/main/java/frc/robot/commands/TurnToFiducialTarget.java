// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.FiducialNotDetectedException;
import frc.robot.subsystems.vision.Limelight;

/**
 * A command that turns the robot towards a target extrapolated from a fiducial. If the fiducial is not seen, the
 * command will wait until it is. The command ends when the robot is facing the target within a given tolerance. If
 * the tolerance is set too small, the command may never end, so it is recommended that this command is run with a
 * Deadline to prevent it from hogging resources from other commands.
 */
public class TurnToFiducialTarget extends CommandBase {

  private Drivetrain drivetrain;
  private Limelight camera;
  private int fiducialId;
  private double turnSpeed, tolerance;

  private Optional<Double> angle = Optional.empty();

  /**
   * Creates a new {@code TurnToFiducialTarget} command.
   *
   * @param drivetrain The drivetrain which will be commanded to rotate.
   * @param camera The camera from which fiducial position data will be collected.
   * @param fiducialId The ID fiducial specifically being searched for.
   * @param turnSpeed The speed at which the robot should turn in either direction when correcting angle.
   * @param tolerance The acceptable deviation from "straight-on" the target, in degrees.
   * */
  public TurnToFiducialTarget(Drivetrain drivetrain, Limelight camera, int fiducialId, double turnSpeed, double tolerance) {
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.camera = camera;

    this.fiducialId = fiducialId;
    this.turnSpeed = turnSpeed;
    this.tolerance = tolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Limelight.Result currentResult = null;
    try {
      currentResult = camera.getLatestResult();
    } catch (Exception e) {
      e.printStackTrace();
    }

    if (currentResult == null) {
      angle = Optional.empty();
      drivetrain.stop();
    } else {
      try {
        Limelight.Result.Fiducial fiducial = currentResult.getFiducialWithId(fiducialId);
        double correctionAngle = Units.radiansToDegrees(-fiducial.getAngleInCameraView().getZ());
        angle = Optional.of(-correctionAngle);

        if (correctionAngle > 0) {
          drivetrain.driveArcadeStyle(0, turnSpeed);
        } else {
          drivetrain.driveArcadeStyle(0, -turnSpeed);
        }
      } catch (FiducialNotDetectedException e) {
        drivetrain.stop();
        angle = Optional.empty();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (angle.isEmpty()) {
      return false;
    }
    return Math.abs(angle.get()) < tolerance;
  }
}
