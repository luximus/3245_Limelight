// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.DebugUtil;

/**
 * A TargetInfo2D holds information about a 2D target detected by the vision system.
 */
public class TargetInfo2D {

  private final long tagID;
  private final Pose2d cameraPose;
  private final double targetArea;

  /**
   * Construct a new TargetInfo2D.
   * @param tagID ID of the reference AprilTag
   * @param targetName name of the target
   * @param cameraPose pose of the target relative to the camera (X+ is right, Y+ is down)
   * @param robotPose pose of the target relative to the robot (X+ is forward, Y+ is rightward)
   */
  public TargetInfo2D(final long tagID, final Pose2d cameraPose, final double targetArea) {
    this.tagID = tagID;
    this.cameraPose = cameraPose;
    this.targetArea = targetArea;
  }

  /**
   * The ID of the AprilTag used as reference to calculate the target's pose.
   */
  public long getAprilTagID() {
    return tagID;
  }

  /**
   * The pose of the target relative to the camera. X+ is rightward and Y+ is downward when looking
   * through the view of the camera.
   */
  public Pose2d getPoseRelativeToCamera() {
    return cameraPose;
  }

  /**
   * The percent of the camera's view the target occupies.
   */
  public double getTargetArea() {
    return targetArea;
  }

  @Override
  public String toString() {
    return String.format("Target seen w/ tag #%d in camera at %s (covers %3.1f% of camera)",
                         tagID,
                         DebugUtil.formatToString(cameraPose),
                         targetArea * 100);
  }
}
