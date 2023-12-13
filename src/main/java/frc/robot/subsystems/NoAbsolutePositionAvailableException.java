package frc.robot.subsystems;

public class NoAbsolutePositionAvailableException extends Exception {
  public NoAbsolutePositionAvailableException() {}

  public NoAbsolutePositionAvailableException(String message) {
    super(message);
  }

  public NoAbsolutePositionAvailableException(Throwable cause) {
    super(cause);
  }

  public NoAbsolutePositionAvailableException(String message, Throwable cause) {
    super(message, cause);
  }
}
