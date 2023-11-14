// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.EnumSet;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DebugUtil;

/** An interface to a Limelight camera. */
public class Limelight extends SubsystemBase {
  public enum SmartDashboardPublishing {
    TARGET_2D, TARGET_3D, CAMERA_POSE, ROBOT_POSE
  }
  private static final long LONG_SENTINEL = -1;
  private static final double DOUBLE_SENTINEL = Double.NaN;
  private static final double[] DOUBLE6_SENTINEL;
  private static final double[] DOUBLE7_SENTINEL;

  static {
    double[] nans6 = new double[6];
    double[] nans7 = new double[7];
    Arrays.fill(nans6, DOUBLE_SENTINEL);
    Arrays.fill(nans7, DOUBLE_SENTINEL);
    DOUBLE6_SENTINEL = nans6;
    DOUBLE7_SENTINEL = nans7;
  }

  private final NetworkTable netTable;

  private IntegerSubscriber tidSubscriber;
  private DoubleSubscriber txSubscriber, tySubscriber, taSubscriber;
  private DoubleArraySubscriber targetPoseInCameraSpaceSubscriber, targetPoseInRobotSpaceSubscriber;
  private DoubleArraySubscriber robotPoseInFieldSpaceSubscriber, robotPoseInDriverStationSpaceSubscriber;
  private DoubleArraySubscriber cameraPoseInRobotSpaceSubscriber;
  private DoubleSubscriber tlSubscriber, clSubscriber;

  private DoubleArrayPublisher cameraPoseInRobotSpacePublisher;

  private IntegerEntry pipelineEntry;

  private final EnumSet<SmartDashboardPublishing> publishingOptions;

  public Limelight(String tableName, SmartDashboardPublishing... publishingOptions) {
    netTable = NetworkTableInstance.getDefault().getTable(tableName);

    tidSubscriber = netTable.getIntegerTopic("tid").subscribe(LONG_SENTINEL);

    txSubscriber = netTable.getDoubleTopic("tx").subscribe(DOUBLE_SENTINEL);
    tySubscriber = netTable.getDoubleTopic("ty").subscribe(DOUBLE_SENTINEL);
    taSubscriber = netTable.getDoubleTopic("ta").subscribe(DOUBLE_SENTINEL);

    targetPoseInCameraSpaceSubscriber = netTable.getDoubleArrayTopic("targetpose_cameraspace").subscribe(DOUBLE6_SENTINEL);
    targetPoseInRobotSpaceSubscriber = netTable.getDoubleArrayTopic("targetpose_robotspace").subscribe(DOUBLE6_SENTINEL);

    robotPoseInFieldSpaceSubscriber = netTable.getDoubleArrayTopic("botpose").subscribe(DOUBLE7_SENTINEL);
    initRobotPoseInDriverStationSpaceSubscriber();

    cameraPoseInRobotSpaceSubscriber = netTable.getDoubleArrayTopic("camerapose_robotspace").subscribe(DOUBLE6_SENTINEL);

    tlSubscriber = netTable.getDoubleTopic("tl").subscribe(DOUBLE_SENTINEL);
    clSubscriber = netTable.getDoubleTopic("cl").subscribe(DOUBLE_SENTINEL);

    cameraPoseInRobotSpacePublisher = netTable.getDoubleArrayTopic("camerapose_robotspace_set").publish();

    pipelineEntry = netTable.getIntegerTopic("pipeline").getEntry(LONG_SENTINEL);

    this.publishingOptions = EnumSet.copyOf(Arrays.asList(publishingOptions));
  }

  private boolean initRobotPoseInDriverStationSpaceSubscriber() {
    switch (DriverStation.getAlliance()) {
      case Blue:
        robotPoseInDriverStationSpaceSubscriber = netTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(DOUBLE7_SENTINEL);
        return true;
      case Red:
        robotPoseInDriverStationSpaceSubscriber = netTable.getDoubleArrayTopic("botpose_wpired").subscribe(DOUBLE7_SENTINEL);
        return true;
      default:
        robotPoseInDriverStationSpaceSubscriber = null;
        return false;
    }
  }

  /**
   * Get information about the current Limelight target in the camera's view.
   * @return a {@link TargetInfo2D} containing target information wrapped in an optional
   */
  public Optional<TargetInfo2D> getCurrent2DTargetInfo() {
    var tagID = getAprilTagID();
    var targetPose = getTargetPose2DInCamera();
    var targetArea = getTargetAreaInCamera();
    if (tagID.isPresent() && targetPose.isPresent() && targetArea.isPresent()) {
      return Optional.of(new TargetInfo2D(tagID.get(), targetPose.get(), targetArea.get()));
    } else {
      return Optional.empty();
    }
  }

  /**
   * Get information about the current Limelight target in three-dimensional space.
   * @return a {@link TargetInfo3D} containing target information wrapped in an optional
   */
  public Optional<TargetInfo3D> getCurrent3DTargetInfo() {
    var tagID = getAprilTagID();
    var cameraPose = getTargetPose3DInCameraSpace();
    var robotPose = getTargetPose3DInRobotSpace();
    if (tagID.isPresent() && cameraPose.isPresent()) {
      if (robotPose.isPresent()) {
        return Optional.of(new TargetInfo3D(tagID.get(), cameraPose.get(), robotPose.get()));
      } else {
        return Optional.of(new TargetInfo3D(tagID.get(), cameraPose.get()));
      }
    } else {
      return Optional.empty();
    }
  }

  /**
   * Whether the Limelight currently sees a target.
   * @return true if the Limelight sees the target
   */
  public boolean seesTarget() {
    return getEntryIfPopulated(tidSubscriber).isPresent();
  }

  /**
   * The ID of the AprilTag currently seen by the Limelight.
   * @return ID of the AprilTag currently seen, or else Optional.empty()
   */
  public Optional<Long> getAprilTagID() {
    return getEntryIfPopulated(tidSubscriber);
  }

  /**
   * The current pose of the target in the camera reference frame. Rotation information is not
   * calculated due to a limitation of the algorithm.
   *
   * Camera's view:
   *   X+: rightward
   *   Y+: downward
   *   origin: center of camera view
   * @return current pose of the target in the camera's frame, or else Optional.empty()
   */
  public Optional<Pose2d> getTargetPose2DInCamera() {
    Optional<Double> targetX = getEntryIfPopulated(txSubscriber);
    Optional<Double> targetY = getEntryIfPopulated(tySubscriber);
    if (targetX.isPresent() && targetY.isPresent()) {
      return Optional.of(new Pose2d(targetX.get(), targetY.get(), new Rotation2d()));
    } else {
      return Optional.empty();
    }
  }

  /**
   * The area the target consumes in the camera view.
   * @return the area the target consumes in the camera view, or else Optional.empty()
   */
  public Optional<Double> getTargetAreaInCamera() {
    return getEntryIfPopulated(taSubscriber);
  }

  /**
   * The three-dimensional pose of the target in the camera's reference frame. This will return
   * Optional.empty() if the current pipeline is not configured to use 3D tracking.
   *
   * Camera's reference frame:
   *   X+: rightward
   *   Y+: downward
   *   Z+: forward
   *   origin: center of camera
   * @return Pose3d of target in camera's frame, or else Optional.empty()
   */
  public Optional<Pose3d> getTargetPose3DInCameraSpace() {
    Optional<double[]> result = getEntryIfPopulated(targetPoseInCameraSpaceSubscriber);
    if (result.isPresent()) {
      return Optional.of(getPose3dFromTransformArray(result.get()));
    } else {
      return Optional.empty();
    }
  }

  /**
   * The three-dimensional pose of the target in the robot's reference frame. This will return
   * Optional.empty() if the current pipeline is not configured to use 3D tracking.
   *
   * Robot's reference frame:
   *   X+: forward
   *   Y+: rightward
   *   Z+: upward, towards the sky
   *   origin: center of the robot
   * @return Pose3d of target in robot's frame, or else Optional.empty()
   */
  public Optional<Pose3d> getTargetPose3DInRobotSpace() {
    Optional<double[]> result = getEntryIfPopulated(targetPoseInRobotSpaceSubscriber);
    if (result.isPresent()) {
      return Optional.of(getPose3dFromTransformArray(result.get()));
    } else {
      return Optional.empty();
    }
  }

  /**
   * The two-dimensional pose of the robot in the field's reference frame. This will return
   * Optional.empty() if the current pipeline is not configured to use 3D tracking.
   *
   * Field's reference frame:
   *   X+: long edge
   *   Y+: short edge
   *   origin: center of the field
   * @return Pose2d of robot in field's frame, or else Optional.empty()
   */
  public Optional<Pose2d> getRobotPose2DInFieldSpace() {
    Optional<double[]> result = getEntryIfPopulated(robotPoseInFieldSpaceSubscriber);
    if (result.isPresent()) {
      return Optional.of(getPose3dFromTransformArray(result.get()).toPose2d());
    } else {
      return Optional.empty();
    }
  }

  /**
   * The three-dimensional pose of the robot in the field's reference frame. This will return
   * Optional.empty() if the current pipeline is not configured to use 3D tracking.
   *
   * Field's reference frame:
   *   X+: long edge
   *   Y+: short edge
   *   Z+: up, towards the sky
   *   origin: center of the field
   * @return Pose3d of robot in field's frame, or else Optional.empty()
   */
  public Optional<Pose3d> getRobotPose3DInFieldSpace() {
    Optional<double[]> result = getEntryIfPopulated(robotPoseInFieldSpaceSubscriber);
    if (result.isPresent()) {
      return Optional.of(getPose3dFromTransformArray(result.get()));
    } else {
      return Optional.empty();
    }
  }

  /**
   * The two-dimensional pose of the robot in the driver station's reference frame. This will return
   * Optional.empty() if the current pipeline is not configured to use 3D tracking or if alliance
   * is not configured in Driver Station.
   *
   * Driver station's reference frame:
   *   X+: same edge as driver station's
   *   Y+: away from driver station
   *   origin: center of the field
   * @return Pose2d of robot in driver station's frame, or else Optional.empty()
   */
  public Optional<Pose2d> getRobotPose2DInDriverStationSpace() {
    if (robotPoseInDriverStationSpaceSubscriber == null) {
      if (initRobotPoseInDriverStationSpaceSubscriber()) {
        return getRobotPose2DInDriverStationSpace();
      } else {
        return Optional.empty();
      }
    }

    Optional<double[]> result = getEntryIfPopulated(robotPoseInDriverStationSpaceSubscriber);
    if (result.isPresent()) {
      return Optional.of(getPose3dFromTransformArray(result.get()).toPose2d());
    } else {
      return Optional.empty();
    }
  }

  /**
   * The three-dimensional pose of the robot in the driver station's reference frame. This will return
   * Optional.empty() if the current pipeline is not configured to use 3D tracking or if alliance
   * is not configured in Driver Station.
   *
   * Driver station's reference frame:
   *   X+: same edge as driver station's
   *   Y+: away from driver station
   *   Z+: up, towards the sky
   *   origin: center of the field
   * @return Pose3d of robot in driver station's frame, or else Optional.empty()
   */
  public Optional<Pose3d> getRobotPose3DInDriverStationSpace() {
    if (robotPoseInDriverStationSpaceSubscriber == null) {
      if (initRobotPoseInDriverStationSpaceSubscriber()) {
        return getRobotPose3DInDriverStationSpace();
      } else {
        return Optional.empty();
      }
    }

    Optional<double[]> result = getEntryIfPopulated(robotPoseInDriverStationSpaceSubscriber);
    if (result.isPresent()) {
      return Optional.of(getPose3dFromTransformArray(result.get()));
    } else {
      return Optional.empty();
    }
  }

  /**
   * The three-dimensional pose of the camera in the robot's reference frame. This value is likely
   * constant, as its value is controlled by the user. This will return Optional.empty() if the
   * current pipeline is not configured to use 3D tracking.
   *
   * Robot's reference frame:
   *   X+: forward
   *   Y+: rightward
   *   Z+: upward, towards the sky
   *   origin: center of the robot
   * @return Pose3d of camera in robot's frame, or else Optional.empty()
   */
  public Optional<Pose3d> getCameraPose3DInRobotSpace() {
    Optional<double[]> result = getEntryIfPopulated(cameraPoseInRobotSpaceSubscriber);
    if (result.isPresent()) {
      return Optional.of(getPose3dFromTransformArray(result.get()));
    } else {
      return Optional.empty();
    }
  }

  /**
   * Set the pose of the camera in the robot's reference frame.
   *
   * Robot's reference frame:
   *   X+: forward
   *   Y+: rightward
   *   Z+: upward, towards the sky
   *   origin: center of the robot
   * @param cameraPose new camera pose
   */
  public void setCameraPose3DInRobotSpace(Pose3d cameraPose) {
    cameraPoseInRobotSpacePublisher.set(getTransformArrayFromPose3d(cameraPose));
  }

  /**
   * The latency the image recognition pipeline contributes.
   * @return pipeline-added latency or else Optional.empty()
   */
  public Optional<Double> getPipelineLatency() {
    return getEntryIfPopulated(tlSubscriber);
  }

  /**
   * The latency associated with the camera itself.
   * @return camera latency or else Optional.empty()
   */
  public Optional<Double> getCameraLatency() {
    return getEntryIfPopulated(clSubscriber);
  }

  /**
   * The total latency of the recorded data.
   * @return total latency or else Optional.empty()
   */
  public Optional<Double> getTotalLatency() {
    Optional<Double> tl = getEntryIfPopulated(tlSubscriber);
    Optional<Double> cl = getEntryIfPopulated(clSubscriber);

    if (tl.isPresent() && cl.isPresent()) {
      return Optional.of(tl.get() + cl.get());
    } else {
      return Optional.empty();
    }
  }

  /**
   * The current active pipeline.
   * @return current active pipeline or else Optional.empty()
   */
  public Optional<Long> getActivePipeline() {
    return getEntryIfPopulated(pipelineEntry);
  }

  /**
   * Set the currently active pipeline.
   * @param pipeline the pipeline to set to
   * @throws IllegalArgumentException pipeline is not from 0 to 9, inclusive
   */
  public void setActivePipeline(long pipeline) throws IllegalArgumentException {
    if (pipeline < 0 || pipeline > 9) {
      throw new IllegalArgumentException("Pipeline must be between 0 and 9, inclusive");
    }
    pipelineEntry.set(pipeline);
  }

  /**
   * Convert a transform array into a Pose3d.
   * @param transform transform array
   * @return a Pose3d equivalent to the transform
   */
  private static Pose3d getPose3dFromTransformArray(double[] transform) {
    Translation3d translation = new Translation3d(transform[0], transform[1], transform[2]);
    Rotation3d rotation = new Rotation3d(transform[3], transform[4], transform[5]);
    return new Pose3d(translation, rotation);
  }

  /**
   * Convert a Pose3d into a transform array.
   * @param pose a Pose3d
   * @return a transform array equivalent to the pose
   */
  private static double[] getTransformArrayFromPose3d(Pose3d pose) {
    Rotation3d rotation = pose.getRotation();
    return new double[]{pose.getX(), pose.getY(), pose.getZ(), rotation.getX(), rotation.getY(), rotation.getZ()};
  }

  private static Optional<Long> getEntryIfPopulated(IntegerSubscriber subscriber) {
    long value = subscriber.get(LONG_SENTINEL);
    if (value == LONG_SENTINEL) {
      return Optional.empty();
    } else {
      return Optional.of(value);
    }
  }

  private static Optional<Double> getEntryIfPopulated(DoubleSubscriber subscriber) {
    double value = subscriber.get(DOUBLE_SENTINEL);
    if (value == DOUBLE_SENTINEL) {
      return Optional.empty();
    } else {
      return Optional.of(value);
    }
  }

  private static Optional<double[]> getEntryIfPopulated(DoubleArraySubscriber subscriber) {
    double[] result = subscriber.get(new double[]{DOUBLE_SENTINEL});
    for (double value : result) {
      if (value != DOUBLE_SENTINEL) {
        return Optional.of(result);
      }
    }
    return Optional.empty();
  }

  @Override
  public void periodic() {
    if (publishingOptions.contains(SmartDashboardPublishing.TARGET_2D)) {
      Optional<TargetInfo2D> targetInfo2D = getCurrent2DTargetInfo();
      if (targetInfo2D.isPresent()) {
        SmartDashboard.putString("Current target", targetInfo2D.toString());
      } else {
        SmartDashboard.putString("Current target", "None");
      }
    }

    if (publishingOptions.contains(SmartDashboardPublishing.TARGET_3D)) {
      Optional<TargetInfo3D> targetInfo3D = getCurrent3DTargetInfo();
      if (targetInfo3D.isPresent()) {
        SmartDashboard.putString("Current 3D target", targetInfo3D.toString());
      } else {
        SmartDashboard.putString("Current 3D target", "None");
      }
    }

    if (publishingOptions.contains(SmartDashboardPublishing.CAMERA_POSE)) {
      Optional<Pose3d> cameraPose = getCameraPose3DInRobotSpace();
      if (cameraPose.isPresent()) {
        SmartDashboard.putString("Camera pose in robot's frame", DebugUtil.formatToString(cameraPose.get()));
      } else {
        SmartDashboard.putString("Camera pose in robot's frame", "Unknown");
      }
    }

    if (publishingOptions.contains(SmartDashboardPublishing.ROBOT_POSE)) {
      Optional<Pose3d> robotPose = getRobotPose3DInFieldSpace();
      if (robotPose.isPresent()) {
        SmartDashboard.putString("Robot 2D pose", DebugUtil.formatToString(robotPose.get().toPose2d()));
        SmartDashboard.putString("Robot 3D pose", DebugUtil.formatToString(robotPose.get()));
      } else {
        SmartDashboard.putString("Robot 2D pose", "Unknown");
        SmartDashboard.putString("Robot 3D pose", "Unknown");
      }
    }
  }
}
