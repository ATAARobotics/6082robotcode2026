// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    public static final int FrontLeftId = 12;
    public static final int BackLeftId = 11;
    public static final int FrontRightId = 14;
    public static final int BackRightId = 13;

    public static final double kTrackWidthMeters = Units.inchesToMeters(24.5);

    // meters per motor rotation = wheelDiameterMeters * Math.PI / gearRatio
    public static final double kMetersPerRotation = 0.0566;
  }

  public static class VisionConstants {
    public static final String kLimelightName = "limelight";

    public static final double kCameraForwardMeters = 0.0;
    public static final double kCameraSideMeters = 0.0;
    public static final double kCameraUpMeters = 0.0;

    public static final double kCameraRollDegrees = 0.0;
    public static final double kCameraPitchDegrees = 0.0;
    public static final double kCameraYawDegrees = 0.0;
  }

  public static class StartPoses {
    public static final Pose2d kBlueStart1 = new Pose2d(0.0, 0.0, new Rotation2d());
    public static final Pose2d kBlueStart2 = new Pose2d(0.0, 0.0, new Rotation2d());
    public static final Pose2d kBlueStart3 = new Pose2d(0.0, 0.0, new Rotation2d());
    public static final Pose2d kRedStart1 = new Pose2d(0.0, 0.0, new Rotation2d());
    public static final Pose2d kRedStart2 = new Pose2d(0.0, 0.0, new Rotation2d());
    public static final Pose2d kRedStart3 = new Pose2d(0.0, 0.0, new Rotation2d());
  }
}
