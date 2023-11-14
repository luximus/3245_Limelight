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

  // TODO: CAN network
  private static final int LEFT_REAR_MOTOR_ID = -1;
  private static final int LEFT_FRONT_MOTOR_ID = -1;
  private static final int RIGHT_REAR_MOTOR_ID = -1;
  private static final int RIGHT_FRONT_MOTOR_ID = -1;

  WPI_TalonSRX leftRearMotor = new WPI_TalonSRX(LEFT_REAR_MOTOR_ID);
  WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(LEFT_FRONT_MOTOR_ID);
  WPI_TalonSRX rightRearMotor = new WPI_TalonSRX(RIGHT_REAR_MOTOR_ID);
  WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(RIGHT_FRONT_MOTOR_ID);

  MotorControllerGroup leftMotors = new MotorControllerGroup(leftFrontMotor, leftRearMotor);
  MotorControllerGroup rightMotors = new MotorControllerGroup(rightFrontMotor, rightRearMotor);

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
