// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpinnyIndicator extends SubsystemBase {

  private static final int MOTOR_ID = 0;
  private static final double MOTOR_SPIN_SPEED = 0.85;

  Talon motor = new Talon(MOTOR_ID);

  /** Creates a new SpinnyIndicator. */
  public SpinnyIndicator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin() {
    motor.set(MOTOR_SPIN_SPEED);
  }

  public void stop() {
    motor.set(0.0);
  }
}
