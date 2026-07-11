// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
  }

  public static class IntakeConstants {
    public static final int intakeMotorId = 15;

    public static final double intakeSpeed = 0.6;
  }

  public static class DrivetrainConstants {
    public static final int FrontLeftId = 12;
    public static final int BackLeftId = 11;
    public static final int FrontRightId = 14;
    public static final int BackRightId = 13;
    public static final int PigeonId = 20;

    // Max acceleration per second for the drive limiter.
    public static final double driveMaxRatePerSec = 14.0;

    // Power-curve exponent for the drive limiter.
    public static final double driveCurveExponent = 3;

    // Maximum power for drive
    public static final double driveMaxPower = 0.8;

    // Max acceleration per second for the turn limiter.
    public static final double turnMaxRatePerSec = 14.0;

    // Power-curve exponent for the turn limiter.
    public static final double turnCurveExponent = 4;

    // Maximum power for turn
    public static final double turnMaxPower = 0.8;

    public static final double trackWidthMeters = Units.inchesToMeters(24.5);

    // meters per motor rotation = wheelDiameterMeters * Math.PI / gearRatio
    public static final double metersPerRotation = 0.04849963145726829;
  }

  public static class VisionConstants {
    public static final String limelightName = "limelight";
  }

  public static class StartPoses {
    public static final Pose2d blueStart1 = new Pose2d(0.0, 0.0, new Rotation2d());
    public static final Pose2d blueStart2 = new Pose2d(0.0, 0.0, new Rotation2d());
    public static final Pose2d blueStart3 = new Pose2d(0.0, 0.0, new Rotation2d());
    public static final Pose2d redStart1 = new Pose2d(0.0, 0.0, new Rotation2d());
    public static final Pose2d redStart2 = new Pose2d(0.0, 0.0, new Rotation2d());
    public static final Pose2d redStart3 = new Pose2d(0.0, 0.0, new Rotation2d());
  }
}
