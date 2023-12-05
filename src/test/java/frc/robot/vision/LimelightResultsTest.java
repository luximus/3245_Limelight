package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.junit.jupiter.api.Test;

import com.fasterxml.jackson.core.JsonProcessingException;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.Full3DTargetingDisabledException;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.MegaTagDisabledException;
import frc.robot.subsystems.vision.Limelight.Result.Fiducial;

public class LimelightResultsTest {

  private static final Path resourcePath = Paths.get("src", "test", "resources");

  private final String noTargetsJsonSample;
  private final String one2dTargetJsonSample;
  private final String one3dTargetJsonSample;
  private final String two3dTargetsJsonSample;

  public LimelightResultsTest() throws IOException {
    noTargetsJsonSample = Files.readString(resourcePath.resolve("limelight_output_no_targets.json"));
    one2dTargetJsonSample = Files.readString(resourcePath.resolve("limelight_output_1_2d_target.json"));
    one3dTargetJsonSample = Files.readString(resourcePath.resolve("limelight_output_1_3d_target.json"));
    two3dTargetsJsonSample = Files.readString(resourcePath.resolve("limelight_output_2_3d_targets.json"));
  }

  @Test
  void hasTargetsWhenExpected() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);

    assertTrue(result::hasTargets, "hasTargets property claims targets were not found when some should be");
  }

  @Test
  void doesNotHaveTargetsWhenExpected() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(noTargetsJsonSample);

    assertFalse(result::hasTargets, "hasTargets property claims targets were found when none should be");
  }

  @Test
  void fiducialArrayHasOneTargetWhenExpected() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);

    assertEquals(result.getFoundFiducials().length, 1, "Expected 1 found fiducial");
  }

  @Test
  void fiducialArrayHasTwoTargetsWhenExpected() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(two3dTargetsJsonSample);

    assertEquals(result.getFoundFiducials().length, 2, "Expected 2 found fiducials");
  }

  @Test
  void fiducialArrayDoesNotHaveTargetsWhenExpected() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(noTargetsJsonSample);

    assertEquals(result.getFoundFiducials().length, 0, "Expected no found fiducials");
  }

  @Test
  void gotExpectedPipelineId() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);

    assertEquals(result.getPipelineId(), 0, "Expected pipeline 0");
  }

  @Test
  void gotTypicalTotalLatency() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);

    assertTrue(result.getTotalLatency() > (12.496978759765625 + 13.945555686950684), "Target latency was less than expected (got " + result.getTotalLatency() + ")");
  }

  @Test
  void failGettingMegaTag2dPoseWithoutMegaTag() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);

    assertThrows(MegaTagDisabledException.class, result::getRobotPose2dInFieldSpace);
  }

  @Test
  void failGettingMegaTag3dPoseWithoutMegaTag() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);

    assertThrows(MegaTagDisabledException.class, result::getRobotPose3dInFieldSpace);
  }

  @Test
  void gotExpectedMegaTag2dPose() throws JsonProcessingException, MegaTagDisabledException {
    Limelight.Result result = Limelight.Result.createFromJson(one3dTargetJsonSample);

    assertEquals(result.getRobotPose2dInFieldSpace(),
                 new Pose3d(0, 0, 0,
                            new Rotation3d(Units.degreesToRadians(89.9999239850482),
                                           Units.degreesToRadians(-89.9999239850482),
                                           Units.degreesToRadians(8.537736462515939e-07)))
                  .toPose2d());
  }

  @Test
  void gotExpectedMegaTag3dPose() throws JsonProcessingException, MegaTagDisabledException {
    Limelight.Result result = Limelight.Result.createFromJson(one3dTargetJsonSample);

    assertEquals(result.getRobotPose3dInFieldSpace(),
                 new Pose3d(0, 0, 0,
                            new Rotation3d(Units.degreesToRadians(89.9999239850482),
                                           Units.degreesToRadians(-89.9999239850482),
                                           Units.degreesToRadians(8.537736462515939e-07))));
  }

  @Test
  void gotExpectedFiducialId() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertEquals(fiducial.getId(), 0);
  }

  @Test
  void gotExpectedFiducialPosition() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertEquals(fiducial.getAngleInCameraView(), new Rotation3d(0, Units.degreesToRadians(-10.46579991906529), Units.degreesToRadians(-9.52471525306504)));
  }

  @Test
  void gotExpectedFiducialArea() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertEquals(fiducial.getAreaOfImageConsumed(), 0.02870107628405094);
  }

  @Test
  void failGettingFiducial2dPoseWhenFull3dTargetingDisabled() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertThrows(Full3DTargetingDisabledException.class, fiducial::getPose2dInCameraSpace);
  }

  @Test
  void gotExpectedFiducial2dPose() throws JsonProcessingException, Full3DTargetingDisabledException {
    Limelight.Result result = Limelight.Result.createFromJson(one3dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertEquals(fiducial.getPose2dInCameraSpace(),
                 new Pose3d(-0.004804806334926148, -0.02760404426781503, 0.7949918128199002,
                            new Rotation3d(Units.degreesToRadians(-36.73993499810669),
                                           Units.degreesToRadians(-0.394281999367109),
                                           Units.degreesToRadians(9.432735645196432)))
                  .toPose2d());
  }

  @Test
  void failGettingFiducial3dPoseWhenFull3dTargetingDisabled() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertThrows(Full3DTargetingDisabledException.class, fiducial::getPose3dInCameraSpace);
  }

  @Test
  void gotExpectedFiducial3dPose() throws JsonProcessingException, Full3DTargetingDisabledException {
    Limelight.Result result = Limelight.Result.createFromJson(one3dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertEquals(fiducial.getPose3dInCameraSpace(),
                 new Pose3d(-0.004804806334926148, -0.02760404426781503, 0.7949918128199002,
                            new Rotation3d(Units.degreesToRadians(-36.73993499810669),
                                           Units.degreesToRadians(-0.394281999367109),
                                           Units.degreesToRadians(9.432735645196432))));
  }

  @Test
  void failGettingFiducial2dPoseInRobotSpaceWhenFull3dTargetingDisabled() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertThrows(Full3DTargetingDisabledException.class, fiducial::getPose2dInRobotSpace);
  }

  @Test
  void gotExpectedFiducial2dPoseInRobotSpace() throws JsonProcessingException, Full3DTargetingDisabledException {
    Limelight.Result result = Limelight.Result.createFromJson(one3dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertEquals(fiducial.getPose2dInCameraSpace(),
                 new Pose3d(-0.004804806334926148, -0.02760404426781503, 0.7949918128199002,
                            new Rotation3d(Units.degreesToRadians(-36.73993499810669),
                                           Units.degreesToRadians(-0.394281999367109),
                                           Units.degreesToRadians(9.432735645196432)))
                  .toPose2d());
  }

  @Test
  void failGettingFiducial3dPoseInRobotSpaceWhenFull3dTargetingDisabled() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertThrows(Full3DTargetingDisabledException.class, fiducial::getPose3dInRobotSpace);
  }

  @Test
  void gotExpectedFiducial3dPoseInRobotSpace() throws JsonProcessingException, Full3DTargetingDisabledException {
    Limelight.Result result = Limelight.Result.createFromJson(one3dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertEquals(fiducial.getPose3dInCameraSpace(),
                 new Pose3d(-0.004804806334926148, -0.02760404426781503, 0.7949918128199002,
                            new Rotation3d(Units.degreesToRadians(-36.73993499810669),
                                           Units.degreesToRadians(-0.394281999367109),
                                           Units.degreesToRadians(9.432735645196432))));
  }

  @Test
  void failGettingCamera3dPoseInTargetSpaceWhenFull3dTargetingDisabled() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertThrows(Full3DTargetingDisabledException.class, fiducial::getCameraPose3dInMySpace);
  }

  @Test
  void gotExpectedCamera3dPoseInTargetSpace() throws JsonProcessingException, Full3DTargetingDisabledException {
    Limelight.Result result = Limelight.Result.createFromJson(one3dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertEquals(fiducial.getCameraPose3dInMySpace(),
                 new Pose3d(0.003792927553466286, 0.4967687355603317, -0.6212917905017659,
                            new Rotation3d(Units.degreesToRadians(36.325493706358635),
                                           Units.degreesToRadians(5.939373778660809),
                                           Units.degreesToRadians(-7.351757590936111))));
  }

  @Test
  void failGettingRobot3dPoseInTargetSpaceWhenFull3dTargetingDisabled() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertThrows(Full3DTargetingDisabledException.class, fiducial::getCameraPose3dInMySpace);
  }

  @Test
  void gotExpectedRobot3dPoseInTargetSpace() throws JsonProcessingException, Full3DTargetingDisabledException {
    Limelight.Result result = Limelight.Result.createFromJson(one3dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertEquals(fiducial.getCalculatedRobotPose3dInMySpace(),
                 new Pose3d(0.003792927553466286, 0.4967687355603317, -0.6212917905017659,
                            new Rotation3d(Units.degreesToRadians(36.325493706358635),
                                           Units.degreesToRadians(5.939373778660809),
                                           Units.degreesToRadians(-7.351757590936111))));
  }

  @Test
  void failGettingRobot2dPoseWhenFull3dTargetingDisabled() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertThrows(Full3DTargetingDisabledException.class, fiducial::getCalculatedRobotPose2dInFieldSpace);
  }

  @Test
  void failGettingRobot3dPoseWhenFull3dTargetingDisabled() throws JsonProcessingException {
    Limelight.Result result = Limelight.Result.createFromJson(one2dTargetJsonSample);
    Fiducial fiducial = result.getFoundFiducials()[0];

    assertThrows(Full3DTargetingDisabledException.class, fiducial::getCalculatedRobotPose3dInFieldSpace);
  }
}
