// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.Limelight.Result;

/** Add your docs here. */
public class FollowTrajectory extends SwerveControllerCommand {

  private Limelight camera;

  public FollowTrajectory(Trajectory trajectory,
                          Drivetrain drivetrain,
                          PIDController xController,
                          PIDController yController,
                          ProfiledPIDController thetaController) {
    super(trajectory.relativeTo(drivetrain.getRelativePose()), drivetrain::getRelativePose, Drivetrain.getKinematics(), xController, yController, thetaController,
          drivetrain::setModuleStates, drivetrain);

    camera = null;
  }

  public FollowTrajectory(Trajectory trajectory,
                          Drivetrain drivetrain,
                          Limelight camera,
                          PIDController xController,
                          PIDController yController,
                          ProfiledPIDController thetaController) {
    super(trajectory.relativeTo(drivetrain.getRelativePose()), drivetrain::getRelativePose, Drivetrain.getKinematics(), xController, yController, thetaController,
          drivetrain::setModuleStates, drivetrain, camera);

    this.camera = camera;
  }

  @Override
  public void execute() {
    if (camera != null) {
      try {
        Result result = camera.getLatestResult();
      } catch (IOException e) {

      }
    }
    super.execute();
  }

}
