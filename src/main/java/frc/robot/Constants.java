// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    public static final int FrontLeftId = 12;
    public static final int BackLeftId = 11;
    public static final int FrontRightId = 14;
    public static final int BackRightId = 13;

    // Max acceleration per second for the drive limiter.
    public static final double kDriveMaxRatePerSec = 14.0;

    // Power-curve exponent for the drive limiter.
    public static final double kDriveCurveExponent = 3;

    // Maximum power for drive
    public static final double kDriveMaxPower = 0.8;

    // Max acceleration per second for the turn limiter.
    public static final double kTurnMaxRatePerSec = 14.0;

    // Power-curve exponent for the turn limiter.
    public static final double kTurnCurveExponent = 4;

    // Maximum power for turn
    public static final double kTurnMaxPower = 0.8;
  }
}
