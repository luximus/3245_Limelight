// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.List;

import com.fasterxml.jackson.core.JsonProcessingException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.MegaTagDisabledException;
import frc.robot.subsystems.vision.Limelight.Result;

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Drivetrain drivetrain = new Drivetrain();
  private final Limelight limelight = new Limelight("limelight");

  // The driver's controller
  XboxController driverController = new XboxController(ControlConstants.Teleop.kDriverControllerPort);

  public RobotContainer() {
    Result initialResult;
    try {
      initialResult = limelight.getLatestResult();
    } catch (IOException e) {
      System.err.println("Failed to get absolute robot pose. Falling back to relative pose.");
      return;
    }

    Pose2d initialPose;
    try {
      initialPose = initialResult.getRobotPose2dInFieldSpace();
    } catch (MegaTagDisabledException e) {
      System.err.println("Failed to get absolute robot pose. Falling back to relative pose.");
      return;
    }

    drivetrain.updatePoseWithVisionMeasurement(initialPose, initialResult.getCaptureTimestamp());
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    drivetrain.setDefaultCommand(getDefaultDriveCommand());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(drivetrain::setToXFormation, drivetrain));
  }

  public Command getDefaultDriveCommand() {
    return new RunCommand(() -> {
      drivetrain.drive(
          -MathUtil.applyDeadband(driverController.getLeftY(), ControlConstants.Teleop.kDriveDeadband),
          -MathUtil.applyDeadband(driverController.getLeftX(), ControlConstants.Teleop.kDriveDeadband),
          -MathUtil.applyDeadband(driverController.getRightX(), ControlConstants.Teleop.kDriveDeadband),
          true, true);
    }, drivetrain);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        ControlConstants.Auto.kMaxSpeedMetersPerSecond,
        ControlConstants.Auto.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Drivetrain.getKinematics());

    // The trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        ControlConstants.Auto.kPThetaController, 0, 0, ControlConstants.Auto.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        drivetrain::getRelativePose, // Functional interface to feed supplier
        Drivetrain.getKinematics(),

        // Position controllers
        new PIDController(ControlConstants.Auto.kPXController, 0, 0),
        new PIDController(ControlConstants.Auto.kPYController, 0, 0),
        thetaController,
        drivetrain::setModuleStates,
        drivetrain);

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(drivetrain::stop);
  }
}