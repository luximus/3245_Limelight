// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Add your docs here. */
public final class DebugUtils {

  public static String formatToString(Pose2d pose2d) {
    return String.format("(%3.1f, %3.1f) rotated %5.1f deg",
                         pose2d.getX(),
                         pose2d.getY(),
                         pose2d.getRotation().getDegrees());
  }

  public static String formatToString(Pose3d pose3d) {
    Rotation3d rotation = pose3d.getRotation();
    return String.format("(%3.1f, %3.1f, %3.1f) rotated by (roll %3.1f, pitch %3.1f, yaw %3.1)",
                         pose3d.getX(), pose3d.getY(), pose3d.getZ(),
                         rotation.getX(), rotation.getY(), rotation.getZ());
  }
}
