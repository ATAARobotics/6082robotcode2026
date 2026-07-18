// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
  }

  public static class IntakeConstants {
    public static final int intakeMotorId = 52;

    public static final double intakeSpeed = 0.25;
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
    public static final double driveMaxPower = 0.65;

    // Max acceleration per second for the turn limiter.
    public static final double turnMaxRatePerSec = 14.0;

    // Power-curve exponent for the turn limiter.
    public static final double turnCurveExponent = 4;

    // Maximum power for turn
    public static final double turnMaxPower = 0.8;

    public static final double trackWidthMeters = Units.inchesToMeters(24.5);

    // meters per motor rotation = wheelDiameterMeters * Math.PI / gearRatio
    public static final double metersPerRotation = 0.04849963145726829;

    // meters per second
    public static final double maxDriveSpeed = 3;

    public static final double leftP = 0.3;
    public static final double leftI = 0;
    public static final double leftD = 0;

    public static final double rightP = 0.3;
    public static final double rightI = 0;
    public static final double rightD = 0;
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

  public static class ShooterConstants {
    public static class Shooter {
      public static final int motorId = 50;

      public static final int smartCurrentLimitAmps = 40;

      public static final double setpointLowRpm = 3000.0;
      public static final double setpointMidRpm = 3660.0;
      public static final double setpointHighRpm = 4100.0;

      public static final double pidP = 0.0001;
      public static final double pidI = 0.0;
      public static final double pidD = 0.0;

      public static final double kS = 0.2;
      public static final double kV = 0.00173;

      public static final double allowedErrorRpm = 29.0;

      public static final double minOutput = -1.0;
      public static final double maxOutput = 1.0;

      public static final double indexSpinUpTimeoutSeconds = 3.0;
    }

    public static class Index {
      public static final int motorId = 51;

      public static final int smartCurrentLimitAmps = 40;

      public static final double setpointLowRpm = 3000.0;
      public static final double setpointMidRpm = 3660.0;
      public static final double setpointHighRpm = 4100.0;

      public static final double pidP = 0.0001;
      public static final double pidI = 0.0;
      public static final double pidD = 0.0;

      public static final double kS = 0.15;
      public static final double kV = 0.002;

      public static final double allowedErrorRpm = 52.0;

      public static final double minOutput = -1.0;
      public static final double maxOutput = 1.0;
    }

    public static class JammerConstants {

      public static final int jammerMotorId = 53;

      public static final int smartCurrentLimitAmps = 40;

      public static double speed = 0.5;
    }
  }

  public static RobotConfig robotConfig;
}
