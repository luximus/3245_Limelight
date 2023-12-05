package frc.robot.subsystems.vision;

/**
 * Signals that a fiducial that was queried for was not found.
 */
public class FiducialNotDetectedException extends Exception {
  public FiducialNotDetectedException() {}

  public FiducialNotDetectedException(String message) {
    super(message);
  }

  public FiducialNotDetectedException(Throwable cause) {
    super(cause);
  }

  public FiducialNotDetectedException(String message, Throwable cause) {
    super(message, cause);
  }
}
