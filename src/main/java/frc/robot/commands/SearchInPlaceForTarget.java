// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.Limelight;

/**
 * Turn in place until the requested fiducial is found.
 */
public class SearchInPlaceForTarget extends Command {

  private Drivetrain drivetrain;
  private Limelight camera;

  private int targetFiducialId;
  private boolean found;

  private double turnSpeed;


  /** Create a new {@code SearchInPlaceForTarget}. */
  public SearchInPlaceForTarget(Drivetrain drivetrain, Limelight camera, int fiducialId, double turnSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.camera = camera;
    this.targetFiducialId = fiducialId;
    this.found = false;
    this.turnSpeed = turnSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {
      found = camera.getLatestResult().hasFiducial(targetFiducialId);
    } catch (IOException e) {
      e.printStackTrace();
    }

    drivetrain.driveArcadeStyle(0, turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return found;
  }
}
