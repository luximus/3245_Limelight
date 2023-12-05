package frc.robot.subsystems.vision;

/**
 * Signals that data was attempted to be accessed that requires Full 3D Targeting to be enabled, but Full 3D Targeting
 * was disabled.
 */
public class Full3DTargetingDisabledException extends PipelineFeatureDisabledException {
  public Full3DTargetingDisabledException() {}

  public Full3DTargetingDisabledException(String message) {
    super(message);
  }

  public Full3DTargetingDisabledException(Throwable cause) {
    super(cause);
  }

  public Full3DTargetingDisabledException(String message, Throwable cause) {
    super(message, cause);
  }
}
