// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveArcadeStyle;
import frc.robot.commands.SeekTarget;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.Limelight;

public class RobotContainer {

  private static final String LIMELIGHT_TABLE = "limelight";

  private static final int CONTROLLER_PORT = 0;
  private static final double TELEOP_MAX_SPEED = 0.85;

  Drivetrain drivetrain = new Drivetrain();
  Limelight limelight = new Limelight(LIMELIGHT_TABLE);

  CommandXboxController controller = new CommandXboxController(CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();

    SmartDashboard.putBoolean("Sees target?", false);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(getDefaultDriveCommand());

    new Trigger(limelight::seesTarget)
      .whileTrue(Commands.startEnd(() -> SmartDashboard.putBoolean("Sees target?", true),
                                   () -> SmartDashboard.putBoolean("Sees target?", false)));

    controller.a()
      .whileTrue(getSeekCommand());
  }

  private Command getDefaultDriveCommand() {
    return new DriveArcadeStyle(drivetrain,
                                () -> -controller.getLeftY(),
                                () -> -controller.getRightX(),
                                TELEOP_MAX_SPEED);
  }

  private Command getSeekCommand() {
    return new SeekTarget(drivetrain, limelight, 1, 0.1);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
