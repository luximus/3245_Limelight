// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** An interface to a Limelight camera. */
public class Limelight extends SubsystemBase {

  /**
   * Results of a image-processing pipeline run on a Limelight. A result is calculated from a single snapshot, so
   * information contained in a {@code Result} will be from the same instant in time.
   */
  public static final class Result {

    /**
     * Information about a detected target.
     */
    public static class Target {

    }

    /**
     * Information about a detected fiducial.
     */
    public static final class Fiducial extends Target {

      @JsonProperty("fID")
      private int id;

      @JsonProperty("t6r_fs")
      private double[] calculatedRobotPoseInFieldSpace;

      @JsonProperty("t6r_ts")
      private double[] calculatedRobotPoseInMySpace;

      @JsonProperty("t6c_ts")
      private double[] cameraPoseInMySpace;

      @JsonProperty("t6t_cs")
      private double[] poseInCameraSpace;

      @JsonProperty("t6t_rs")
      private double[] poseInRobotSpace;

      @JsonProperty("ta")
      private double areaOfImage;

      @JsonProperty("tx")
      private double xInCamera;

      @JsonProperty("ty")
      private double yInCamera;

      private Fiducial() {
        calculatedRobotPoseInFieldSpace = new double[6];
        poseInCameraSpace = new double[6];
        poseInRobotSpace = new double[6];
      }

      /**
       * Get the ID of the fiducial.
       */
      public int getId() {
        return id;
      }

      /**
       * <p>Get the 2D pose of the robot in field space calculated using this fiducial.</p>
       *
       * <p>Field space info:
       * <ul>
       *   <li>origin: The center of the field.</li>
       *   <li>+X: Rightward along the long side of the field.</li>
       *   <li>+Y: Upward along the short side of the field.</li>
       * </ul>
       * </p>
       *
       * @throws Full3DTargetingDisabledException Full 3D Targeting was disabled when obtaining this result.
       */
      public Pose2d getCalculatedRobotPose2dInFieldSpace() throws Full3DTargetingDisabledException {
        Pose2d pose = arrayToPose2d(calculatedRobotPoseInFieldSpace);
        if (pose == null) {
          throw new Full3DTargetingDisabledException("Attempted to get robot pose without Full 3D Targeting enabled");
        }
        return pose;
      }

      /**
       * <p>Get the 3D pose of the robot in field space calculated using this fiducial.</p>
       *
       * <p>Field space info:
       * <ul>
       *   <li>origin: The center of the field.</li>
       *   <li>+X: Rightward along the long side of the field.</li>
       *   <li>+Y: Upward along the short side of the field.</li>
       *   <li>+Z: Upward orthogonal to the field's surface.</li>
       * </ul>
       * </p>
       *
       *  @throws Full3DTargetingDisabledException Full 3D Targeting was disabled when obtaining this result.
       */
      public Pose3d getCalculatedRobotPose3dInFieldSpace() throws Full3DTargetingDisabledException {
        Pose3d pose = arrayToPose3d(calculatedRobotPoseInFieldSpace);
        if (pose == null) {
          throw new Full3DTargetingDisabledException("Attempted to get robot pose without Full 3D Targeting enabled");
        }
        return pose;
      }

      /**
       * <p>Get the 3D pose of the robot in this fiducial's space.</p>
       *
       * <p>Target space info:
       * <ul>
       *   <li>origin: The center of the fiducial.</li>
       *   <li>+X: Rightward when looking straight at the target.</li>
       *   <li>+Y: Downward when looking straight at the target.</li>
       *   <li>+Z: Outward orthogonal to the target's plane.</li>
       * </ul>
       * </p>
       *
       *  @throws Full3DTargetingDisabledException Full 3D Targeting was disabled when obtaining this result.
       */
      public Pose3d getCalculatedRobotPose3dInMySpace() throws Full3DTargetingDisabledException {
        Pose3d pose = arrayToPose3d(calculatedRobotPoseInMySpace);
        if (pose == null) {
          throw new Full3DTargetingDisabledException("Attempted to get robot pose without Full 3D Targeting enabled");
        }
        return pose;
      }

      /**
       * <p>Get the 3D pose of the camera in this fiducial's space.</p>
       *
       * <p>Target space info:
       * <ul>
       *   <li>origin: The center of the fiducial.</li>
       *   <li>+X: Rightward when looking straight at the target.</li>
       *   <li>+Y: Downward when looking straight at the target.</li>
       *   <li>+Z: Outward orthogonal to the target's plane.</li>
       * </ul>
       * </p>
       *
       *  @throws Full3DTargetingDisabledException Full 3D Targeting was disabled when obtaining this result.
       */
      public Pose3d getCameraPose3dInMySpace() throws Full3DTargetingDisabledException {
        Pose3d pose = arrayToPose3d(cameraPoseInMySpace);
        if (pose == null) {
          throw new Full3DTargetingDisabledException("Attempted to get camera pose without Full 3D Targeting enabled");
        }
        return pose;
      }

      /**
       * <p>Get the 2D pose of this fiducial in camera space.</p>
       *
       * <p>Camera space info:
       * <ul>
       *   <li>origin: The camera lens.</li>
       *   <li>+X: Rightward when seen from the camera's view.</li>
       *   <li>+Y: Upward when seen from the camera's view.</li>
       * </ul>
       * </p>
       *
       * @throws Full3DTargetingDisabledException Full 3D Targeting was disabled when obtaining this result.
       */
      public Pose2d getPose2dInCameraSpace() throws Full3DTargetingDisabledException {
        Pose2d pose = arrayToPose2d(poseInCameraSpace);
        if (pose == null) {
          throw new Full3DTargetingDisabledException("Attempted to get 3D-derived pose without Full 3D Targeting enabled");
        }
        return pose;
      }

      /**
       * <p>Get the 3D pose of this fiducial in camera space.</p>
       *
       * <p>Camera space info:
       * <ul>
       *   <li>origin: The camera lens.</li>
       *   <li>+X: Rightward when seen from the camera's view.</li>
       *   <li>+Y: Upward when seen from the camera's view.</li>
       *   <li>+Z: Directly forward orthogonal to the camera's front surface.</li>
       * </ul>
       * </p>
       *
       *  @throws Full3DTargetingDisabledException Full 3D Targeting was disabled when obtaining this result.
       */
      public Pose3d getPose3dInCameraSpace() throws Full3DTargetingDisabledException {
        Pose3d pose = arrayToPose3d(poseInCameraSpace);
        if (pose == null) {
          throw new Full3DTargetingDisabledException("Attempted to get 3D pose without Full 3D Targeting enabled");
        }
        return pose;
      }

      /**
       * <p>Get the 2D pose of this fiducial in robot space.</p>
       *
       * <p>Robot space info:
       * <ul>
       *   <li>origin: The center of the robot.</li>
       *   <li>+X: Forward as perceived by the robot.</li>
       *   <li>+Y: Rightward as perceived by the robot.</li>
       * </ul>
       * </p>
       *
       * @throws Full3DTargetingDisabledException Full 3D Targeting was disabled when obtaining this result.
       */
      public Pose2d getPose2dInRobotSpace() throws Full3DTargetingDisabledException {
        Pose2d pose = arrayToPose2d(poseInRobotSpace);
        if (pose == null) {
          throw new Full3DTargetingDisabledException("Attempted to get 3D-derived pose without Full 3D Targeting enabled");
        }
        return pose;
      }

      /**
       * <p>Get the 3D pose of this fiducial in robot space.</p>
       *
       * <p>Robot space info:
       * <ul>
       *   <li>origin: The center of the robot.</li>
       *   <li>+X: Forward as perceived by the robot.</li>
       *   <li>+Y: Rightward as perceived by the robot.</li>
       *   <li>+Z: Upward as perceived by the robot. If the robot is on a slanted surface, this will not be orthogonal
       *       to the field's surface.</li>
       * </ul>
       * </p>
       *
       * @throws Full3DTargetingDisabledException Full 3D Targeting was disabled when obtaining this result.
       */
      public Pose3d getPose3dInRobotSpace() throws Full3DTargetingDisabledException {
        Pose3d pose = arrayToPose3d(poseInRobotSpace);
        if (pose == null) {
          throw new Full3DTargetingDisabledException("Attempted to get 3D pose without Full 3D Targeting enabled");
        }
        return pose;
      }

      /**
       * Get the area of the camera's field of view that this fiducial consumes.
       */
      public double getAreaOfImageConsumed() {
        return areaOfImage;
      }

      /**
       * Get the angle off the set crosshair of the fiducial in the camera view.
       */
      public Rotation3d getAngleInCameraView() {
        return new Rotation3d(0, Units.degreesToRadians(yInCamera), -Units.degreesToRadians(xInCamera));

      }

    }

    private static final ObjectMapper jsonObjectMapper = new ObjectMapper()
                                                           .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

    @JsonProperty("pID")
    private int pipeline;

    @JsonProperty("tl")
    private double targetingLatency;

    @JsonProperty("cl")
    private double captureLatency;

    @JsonProperty("botpose")
    private double[] robotPoseInFieldSpace;

    @JsonProperty("botpose_wpired")
    private double[] robotPoseFromRedDriverStation;

    @JsonProperty("botpose_wpiblue")
    private double[] robotPoseFromBlueDriverStation;

    @JsonProperty("Fiducial")
    private Fiducial[] fiducials;

    private double parseLatency;

    private double captureTimestamp;

    /**
     * Create a new {@code Result} from a JSON object.
     *
     * @param json The JSON object.
     * @return A {@code Result} containing the information in the JSON object.
     * @throws JsonProcessingException The given JSON was invalid.
     */
    public static Result createFromJson(String json) throws JsonProcessingException {
      long start = System.nanoTime();

      Result result = jsonObjectMapper.readValue(json, RawResult.class).getResult();

      long end = System.nanoTime();
      double parseLatency = (end - start) * 1e-6;
      result.setParseLatency(parseLatency);
      result.setCaptureTimestampUsingLatencyValues();

      return result;
    }

    private Result() {
      robotPoseInFieldSpace = new double[6];
      robotPoseFromRedDriverStation = new double[6];
      robotPoseFromBlueDriverStation = new double[6];
    }

    /**
     * Get the ID of the pipeline that captured this result.
     */
    public int getPipelineId() {
      return pipeline;
    }

    /**
     * Get the total latency of this result.
     */
    public double getTotalLatency() {
      return targetingLatency + captureLatency + parseLatency;
    }

    /**
     * Get the time at which this result was captured relative to when the robot started running. This can be used
     * with WPILib's pose estimators such as {@link edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator}
     * when specifying the time stamp for vision measurements.
     */
    public double getCaptureTimestamp() {
      return captureTimestamp;
    }

    private void setCaptureTimestampUsingLatencyValues() {
      captureTimestamp = Timer.getFPGATimestamp() - (getTotalLatency() / 1000);
    }

    private void setParseLatency(double latency) {
      parseLatency = latency;
    }

    /**
     * Whether this result has any found targets.
     */
    public boolean hasTargets() {
      return getFoundFiducials().length > 0;
    }

    /**
     * <p>Get the 2D pose of the robot in field space calculated using all available fiducials.</p>
     *
     * <p>Field space info:
     * <ul>
     *   <li>origin: The center of the field.</li>
     *   <li>+X: Rightward along the long side of the field.</li>
     *   <li>+Y: Upward along the short side of the field.</li>
     * </ul>
     * </p>
     *
     * @throws MegaTagDisabledException MegaTag was disabled when obtaining this result.
     */
    public Pose2d getRobotPose2dInFieldSpace() throws MegaTagDisabledException {
      Pose2d pose = arrayToPose2d(robotPoseInFieldSpace);
      if (pose == null || Arrays.equals(robotPoseInFieldSpace, DOUBLE6_ZERO)) {
        throw new MegaTagDisabledException("Attempted to get robot pose without MegaTag enabled");
      }
      return pose;
    }

    /**
     * <p>Get the 3D pose of the robot in field space calculated using all available fiducials.</p>
     *
     * <p>Field space info:
     * <ul>
     *   <li>origin: The center of the field.</li>
     *   <li>+X: Rightward along the long side of the field.</li>
     *   <li>+Y: Upward along the short side of the field.</li>
     *   <li>+Z: Upward orthogonal to the field's surface.</li>
     * </ul>
     * </p>
     *
     * @throws MegaTagDisabledException MegaTag was disabled when obtaining this result.
     */
    public Pose3d getRobotPose3dInFieldSpace() throws MegaTagDisabledException {
      Pose3d pose = arrayToPose3d(robotPoseInFieldSpace);
      if (pose == null || Arrays.equals(robotPoseInFieldSpace, DOUBLE6_ZERO)) {
        throw new MegaTagDisabledException("Attempted to get robot pose without MegaTag enabled");
      }
      return pose;
    }

    /**
     * <p>Get the 2D pose of the robot in field space oriented from the view of the current alliance's driver stations
     * calculated using all available fiducials.</p>
     *
     * <p>Field space info:
     * <ul>
     *   <li>origin: The center of the field.</li>
     *   <li>+X: Rightward along the side of the field parallel to the driver station lineup.</li>
     *   <li>+Y: Away from the driver stations, perpendicular to their lineup.</li>
     * </ul>
     * </p>
     *
     * @throws MegaTagDisabledException MegaTag was disabled when obtaining this result.
     * @throws NoAllianceException This robot is not currently on an alliance.
     */
    public Pose2d getRobotPose2dFromDriverStation() throws MegaTagDisabledException, NoAllianceException {
      Pose2d pose;
      switch (DriverStation.getAlliance()) {
        case Red:
          pose = arrayToPose2d(robotPoseFromRedDriverStation);
          break;
        case Blue:
          pose = arrayToPose2d(robotPoseFromBlueDriverStation);
          break;
        default:
          throw new NoAllianceException("Attempted to get robot pose from driver station without an alliance set");
      }

      if (pose == null || Arrays.equals(robotPoseFromBlueDriverStation, DOUBLE6_ZERO)  || Arrays.equals(robotPoseFromRedDriverStation, DOUBLE6_ZERO)) {
        throw new MegaTagDisabledException("Attempted to get robot pose without MegaTag enabled");
      }

      return pose;
    }

    /**
     * <p>Get the 3D pose of the robot in field space oriented from the view of the current alliance's driver stations
     * calculated using all available fiducials.</p>
     *
     * <p>Field space info:
     * <ul>
     *   <li>origin: The center of the field.</li>
     *   <li>+X: Rightward along the side of the field parallel to the driver station lineup.</li>
     *   <li>+Y: Away from the driver stations, perpendicular to their lineup.</li>
     *   <li>+Z: Upward orthogonal to the field's surface.</li>
     * </ul>
     * </p>
     *
     * @throws MegaTagDisabledException MegaTag was disabled when obtaining this result.
     * @throws NoAllianceException This robot is not currently on an alliance.
     */
    public Pose3d getRobotPose3dFromDriverStation() throws MegaTagDisabledException, NoAllianceException {
      Pose3d pose;
      switch (DriverStation.getAlliance()) {
        case Red:
          pose = arrayToPose3d(robotPoseFromRedDriverStation);
          break;
        case Blue:
          pose = arrayToPose3d(robotPoseFromBlueDriverStation);
          break;
        default:
          throw new NoAllianceException("Attempted to get robot pose from driver station without an alliance set");
      }

      if (pose == null || Arrays.equals(robotPoseFromBlueDriverStation, DOUBLE6_ZERO) || Arrays.equals(robotPoseFromRedDriverStation, DOUBLE6_ZERO)) {
        throw new MegaTagDisabledException("Attempted to get robot pose without MegaTag enabled");
      }

      return pose;
    }

    /**
     * Get all fiducials detected in the camera view.
     */
    public Fiducial[] getFoundFiducials() {
      return fiducials;
    }

    /**
     * Get a fiducial, if found, with the given ID. If not found, return {@code null}.
     * @param id The ID of the fiducial to search for.
     * @return If found, information about the fiducial with the given id; otherwise {@code null}.
     */
    public Fiducial getFiducialWithId(int id) throws FiducialNotDetectedException {
      for (Fiducial fiducial : fiducials) {
        if (fiducial.getId() == id) {
          return fiducial;
        }
      }
      throw new FiducialNotDetectedException("Did not detect fiducial with ID " + id);
    }

  }

  private static final class RawResult {
    @JsonProperty("Results")
    private Result result;

    private RawResult() {
      result = new Result();
    }

    private Result getResult() {
      return result;
    }
  }

  private static final String JSON_SENTINEL = "";
  private static final long INTEGER_SENTINEL = 0;
  private static final double[] DOUBLE6_SENTINEL;
  private static final double[] DOUBLE6_ZERO = new double[]{0, 0, 0, 0, 0, 0};

  static {
    double[] nans6 = new double[6];
    Arrays.fill(nans6, Double.NaN);
    DOUBLE6_SENTINEL = nans6;
  }

  private final String name;

  private StringSubscriber jsonDataSubscriber;

  private IntegerSubscriber pipelineSubscriber;
  private IntegerPublisher pipelinePublisher;

  private DoubleArraySubscriber cameraPoseInRobotSpaceSubscriber;
  private DoubleArrayPublisher cameraPoseInRobotSpacePublisher;

  private IntegerSubscriber hasTargetSubscriber;

  /**
   * Create a new Limelight interface.
   * @param name The name of the Limelight's network table.
   */
  public Limelight(String name) {
    this.name = name;
    NetworkTable netTable = NetworkTableInstance.getDefault().getTable(name);

    jsonDataSubscriber = netTable.getStringTopic("json").subscribe(JSON_SENTINEL);

    pipelineSubscriber = netTable.getIntegerTopic("getpipe").subscribe(INTEGER_SENTINEL);
    pipelinePublisher = netTable.getIntegerTopic("pipeline").publish();

    cameraPoseInRobotSpaceSubscriber = netTable.getDoubleArrayTopic("camerapose_robotspace").subscribe(DOUBLE6_SENTINEL);
    cameraPoseInRobotSpacePublisher = netTable.getDoubleArrayTopic("camerapose_robotspace_set").publish();

    hasTargetSubscriber = netTable.getIntegerTopic("tv").subscribe(INTEGER_SENTINEL);
  }

  private Optional<String> getJsonDump() {
    String dump = jsonDataSubscriber.get();
    return dump.equals(JSON_SENTINEL) ? Optional.empty() : Optional.of(dump);
  }

  /**
   * Get the result from the most recently processed camera frame.
   * @throws JsonProcessingException The provided JSON was invalid.
   * @throws IOException The JSON dump could not be found.
   */
  public Result getLatestResult() throws JsonProcessingException, IOException {
    long start = System.nanoTime();

    Result result = Result.createFromJson(getJsonDump().orElseThrow(() -> new IOException("JSON dump not found")));

    long end = System.nanoTime();
    double parseLatency = (end - start) * 1e-6;
    result.setParseLatency(parseLatency);
    result.setCaptureTimestampUsingLatencyValues();

    return result;
  }

  /**
   * Get the name of the Limelight's network table.
   */
  public String getName() {
    return name;
  }

  /**
   * Returns {@code true} if the Limelight currently sees a target.
   */
  public boolean seesTarget() {
    return hasTargetSubscriber.get() == 1 ? true : false;
  }

  /**
   * Get the index of the currently active pipeline.
   */
  public long getActivePipeline() {
    return pipelineSubscriber.get();
  }

  /**
   * Set the currently active pipeline.
   * @param pipeline The index of the pipeline to which the Limelight will switch.
   */
  public void setActivePipeline(long pipeline) {
    pipelinePublisher.set(pipeline);
  }

  /**
   * Get the 3D pose of the camera in robot space.
   *
   * <p>Robot space info:
   * <ul>
   *   <li>origin: The center of the robot.</li>
   *   <li>+X: Forward as perceived by the robot.</li>
   *   <li>+Y: Rightward as perceived by the robot.</li>
   *   <li>+Z: Upward as perceived by the robot. If the robot is on a slanted surface, this will not be orthogonal
   *       to the field's surface.</li>
   * </ul>
   * </p>
   *
   * @throws Full3DTargetingDisabledException Full 3D Targeting is currently disabled.
   */
  public Pose3d getCameraPoseInRobotSpace() throws Full3DTargetingDisabledException {
    Pose3d pose = arrayToPose3d(cameraPoseInRobotSpaceSubscriber.get());
    if (pose == null) {
      throw new Full3DTargetingDisabledException("Attempted to get camera pose without Full 3D Targeting enabled");
    }

    return pose;
  }

  /**
   * Set the 3D pose of the camera in robot space.
   *
   * <p>Robot space info:
   * <ul>
   *   <li>origin: The center of the robot.</li>
   *   <li>+X: Forward as perceived by the robot.</li>
   *   <li>+Y: Rightward as perceived by the robot.</li>
   *   <li>+Z: Upward as perceived by the robot. If the robot is on a slanted surface, this will not be orthogonal
   *       to the field's surface.</li>
   * </ul>
   * </p>
   *
   * @param pose The new pose of the camera.
   */
  public void setCameraPoseInRobotSpace(Pose3d pose) {
    cameraPoseInRobotSpacePublisher.set(pose3dToArray(pose));
  }

  private static Pose2d arrayToPose2d(double[] poseData) {
    if (poseData.length < 6) {
      return null;
    }

    return new Pose2d(poseData[0], poseData[1], new Rotation2d(Units.degreesToRadians(poseData[5])));
  }

  private static Pose3d arrayToPose3d(double[] poseData) {
    if (poseData.length < 6) {
      return null;
    }

    return new Pose3d(poseData[0], poseData[1], poseData[2],
                      new Rotation3d(Units.degreesToRadians(poseData[3]),
                                     Units.degreesToRadians(poseData[4]),
                                     Units.degreesToRadians(poseData[5])));
  }

  private static double[] pose3dToArray(Pose3d pose) {
    Rotation3d rotation = pose.getRotation();
    return new double[] {
      pose.getX(),
      pose.getY(),
      pose.getZ(),
      Units.radiansToDegrees(rotation.getX()),
      Units.radiansToDegrees(rotation.getY()),
      Units.radiansToDegrees(rotation.getZ())
    };
  }

}
