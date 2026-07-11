// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
  }

  public static class DrivetrainConstants {
    public static final int FrontLeftId = 12;
    public static final int BackLeftId = 11;
    public static final int FrontRightId = 14;
    public static final int BackRightId = 13;

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
  }

  public static class ShooterConstants {
    public static final int motorId = 20;

    public static final double setpointLowRpm = 1500.0;
    public static final double setpointMidRpm = 3000.0;
    public static final double setpointHighRpm = 4500.0;

    public static final double pidP = 0.00015;
    public static final double pidI = 0.0;
    public static final double pidD = 0.0;

    public static final double kS = 0.05;
    public static final double kV = 0.00018;

    public static final double allowedErrorRpm = 75.0;

    public static final double minOutput = -1.0;
    public static final double maxOutput = 1.0;
  }
}
