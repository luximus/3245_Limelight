package frc.robot.subsystems.vision;


/**
 * Signals that data was attempted to be accessed that requires MegaTag to be enabled, but MegaTag was disabled.
 */
public class MegaTagDisabledException extends PipelineFeatureDisabledException {
  public MegaTagDisabledException() {}

  public MegaTagDisabledException(String message) {
    super(message);
  }

  public MegaTagDisabledException(Throwable cause) {
    super(cause);
  }

  public MegaTagDisabledException(String message, Throwable cause) {
    super(message, cause);
  }
}
