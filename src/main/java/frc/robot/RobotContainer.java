// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveArcadeStyle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.Limelight;

public class RobotContainer {

  private static final String LIMELIGHT_TABLE = "limelight";

  private static final int CONTROLLER_PORT = 0;
  private static final double TELEOP_MAX_SPEED = 0.85;

  Drivetrain drivetrain = new Drivetrain();
  Limelight limelight = new Limelight(LIMELIGHT_TABLE);

  XboxController controller = new XboxController(CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(getDefaultDriveCommand());
  }

  private Command getDefaultDriveCommand() {
    return new DriveArcadeStyle(drivetrain,
                                controller::getLeftY,
                                controller::getRightX,
                                TELEOP_MAX_SPEED);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
