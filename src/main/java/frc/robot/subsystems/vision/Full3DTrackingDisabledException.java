package frc.robot.subsystems.vision;

/**
 * Signals that data was attempted to be accessed that requires Full 3D Tracking to be enabled, but Full 3D Tracking
 * was disabled.
 */
public class Full3DTrackingDisabledException extends PipelineFeatureDisabledException {
  public Full3DTrackingDisabledException() {}

  public Full3DTrackingDisabledException(String message) {
    super(message);
  }

  public Full3DTrackingDisabledException(Throwable cause) {
    super(cause);
  }

  public Full3DTrackingDisabledException(String message, Throwable cause) {
    super(message, cause);
  }
}
