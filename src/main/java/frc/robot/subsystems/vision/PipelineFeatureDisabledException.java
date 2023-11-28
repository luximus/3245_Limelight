package frc.robot.subsystems.vision;


/**
 * Signals that data was attempted to be accessed that requires a certain Limelight pipeline feature to be enabled,
 * but the feature was actually not enabled.
 */
public class PipelineFeatureDisabledException extends Exception {
  public PipelineFeatureDisabledException() {}

  public PipelineFeatureDisabledException(String message) {
    super(message);
  }

  public PipelineFeatureDisabledException(Throwable cause) {
    super(cause);
  }

  public PipelineFeatureDisabledException(String message, Throwable cause) {
    super(message, cause);
  }
}
