// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveArcadeStyle extends CommandBase {

  Drivetrain drivetrain;
  DoubleSupplier forwardSpeedSupplier, turnSpeedSupplier;
  double maxSpeed;

  /** Creates a new DriveArcadeStyle. */
  public DriveArcadeStyle(Drivetrain drivetrain, DoubleSupplier forwardSpeedSupplier, DoubleSupplier turnSpeedSupplier, double maxSpeed) {
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
    this.forwardSpeedSupplier = forwardSpeedSupplier;
    this.turnSpeedSupplier = turnSpeedSupplier;
    this.maxSpeed = maxSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.driveArcadeStyle(forwardSpeedSupplier.getAsDouble(), turnSpeedSupplier.getAsDouble());
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
