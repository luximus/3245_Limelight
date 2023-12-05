// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The drivetrain of the robot.
 */
public final class Drivetrain extends SubsystemBase {

  private static final int LEFT1_MOTOR_ID = 1;
  private static final int LEFT2_MOTOR_ID = 7;
  private static final int RIGHT1_MOTOR_ID = 6;
  private static final int RIGHT2_MOTOR_ID = 23;

  WPI_TalonSRX left1Motor = new WPI_TalonSRX(LEFT1_MOTOR_ID);
  WPI_TalonSRX left2Motor = new WPI_TalonSRX(LEFT2_MOTOR_ID);
  WPI_TalonSRX right1Motor = new WPI_TalonSRX(RIGHT1_MOTOR_ID);
  WPI_TalonSRX right2Motor = new WPI_TalonSRX(RIGHT2_MOTOR_ID);

  MotorControllerGroup leftMotors = new MotorControllerGroup(left2Motor, left1Motor);
  MotorControllerGroup rightMotors = new MotorControllerGroup(right2Motor, right1Motor);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

  /**
   * Create a new Drivetrain.
   */
  public Drivetrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveArcadeStyle(double forwardSpeed, double turnSpeed) {
    differentialDrive.arcadeDrive(forwardSpeed, turnSpeed);
  }

  public void stop() {
    differentialDrive.arcadeDrive(0, 0);
  }
}
