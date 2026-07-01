// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    // Peak rate (units/sec) for the drive (throttle) acceleration limiter.
    public static final double kDriveMaxRatePerSec = 44.0;

    // Power-curve exponent for the drive limiter. Higher = punchier start, softer finish.
    // With the values above, 0 -> 0.6 takes ~1 tick and 0.6 -> 0.85 takes ~0.5s.
    public static final double kDriveCurveExponent = 3.0;

    // Peak rate (units/sec) for the turn (rotation) acceleration limiter.
    public static final double kTurnMaxRatePerSec = 40.0;

    // Power-curve exponent for the turn limiter. Higher = punchier start, softer finish.
    // With the values above, 0 -> 0.5 takes ~1 tick and 0.5 -> 0.75 takes ~0.5s.
    public static final double kTurnCurveExponent = 4.0;
  }
}
