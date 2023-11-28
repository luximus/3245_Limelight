package frc.robot.subsystems.vision;


/**
 * Attempted to access a Limelight interface feature that requires having an alliance set, but no alliance was set.
 */
public class NoAllianceException extends Exception {
  public NoAllianceException() {}

  public NoAllianceException(String message) {
    super(message);
  }

  public NoAllianceException(Throwable cause) {
    super(cause);
  }

  public NoAllianceException(String message, Throwable cause) {
    super(message, cause);
  }
}
