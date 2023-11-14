// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.DebugUtil;

/**
 * A TargetInfo3D holds information about a 3D target detected by the vision system.
 */
public class TargetInfo3D {

  private final long tagID;
  private final Pose3d cameraPose;
  private final Optional<Pose3d> robotPose;

  /**
   * Construct a new TargetInfo3D.
   * @param tagID ID of the reference AprilTag
   * @param cameraPose pose of the target relative to the camera (X+ is right, Y+ is down, Z+ is forward)
   * @param robotPose pose of the target relative to the robot, or else Optional.empty() (X+ is
   *                  forward, Y+ is rightward, Z+ is upward)
   */
  private TargetInfo3D(final long tagID, final Pose3d cameraPose, final Optional<Pose3d> robotPose) {
    this.tagID = tagID;
    this.cameraPose = cameraPose;
    this.robotPose = robotPose;
  }

  /**
   * Construct a new TargetInfo3D.
   * @param tagID ID of the reference AprilTag
   * @param cameraPose pose of the target relative to the camera (X+ is right, Y+ is down, Z+ is forward)
   * @param robotPose pose of the target relative to the robot (X+ is forward, Y+ is rightward, Z+ is upward)
   */
  public TargetInfo3D(final long tagID, final Pose3d cameraPose, final Pose3d robotPose) {
    this(tagID, cameraPose, Optional.of(robotPose));
  }

  /**
   * Construct a new TargetInfo3D.
   * @param tagID ID of the reference AprilTag
   * @param cameraPose pose of the target relative to the camera (X+ is right, Y+ is down, Z+ is forward)
   */
  public TargetInfo3D(final long tagID, final Pose3d cameraPose) {
    this(tagID, cameraPose, Optional.empty());
  }

  /**
   * The ID of the AprilTag used as reference to calculate the target's pose.
   */
  public long getAprilTagID() {
    return tagID;
  }

  /**
   * The pose of the target relative to the camera. X+ is rightward, Y+ is downward, and Z+ is forward
   * when looking through the view of the camera.
   */
  public Pose3d getPoseRelativeToCamera() {
    return cameraPose;
  }

  /**
   * The pose of the target relative to the robot. X+ is forward, Y+ is rightward and Z+ is upward.
   */
  public Optional<Pose3d> getPoseRelativeToRobot() {
    return robotPose;
  }

  @Override
  public String toString() {
    return String.format("Target seen w/ tag #%d in camera at %s (field pose: %s)",
                         tagID,
                         DebugUtil.formatToString(cameraPose),
                         robotPose.isPresent() ? DebugUtil.formatToString(robotPose.get()) : "unknown");
  }
}
