// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.AccelerationLimiter;
import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase {
  private SparkMax frontLeft;
  private SparkMax backLeft;
  private SparkMax frontRight;
  private SparkMax backRight;

  private DifferentialDrive differentialDrive;

  private AccelerationLimiter driveLimiter =
      new AccelerationLimiter(
          Constants.DrivetrainConstants.driveMaxRatePerSec,
          Constants.DrivetrainConstants.driveCurveExponent);
  private AccelerationLimiter turnLimiter =
      new AccelerationLimiter(
          Constants.DrivetrainConstants.turnMaxRatePerSec,
          Constants.DrivetrainConstants.turnCurveExponent);

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    frontLeft = new SparkMax(Constants.DrivetrainConstants.FrontLeftId, MotorType.kBrushless);
    backLeft = new SparkMax(Constants.DrivetrainConstants.BackLeftId, MotorType.kBrushless);
    frontRight = new SparkMax(Constants.DrivetrainConstants.FrontRightId, MotorType.kBrushless);
    backRight = new SparkMax(Constants.DrivetrainConstants.BackRightId, MotorType.kBrushless);

    SparkMaxConfig frontLeftConfig = new SparkMaxConfig();
    frontLeftConfig.follow(Constants.DrivetrainConstants.BackLeftId, false);
    frontLeftConfig.idleMode(IdleMode.kBrake);

    SparkMaxConfig backLeftConfig = new SparkMaxConfig();
    backLeftConfig.idleMode(IdleMode.kBrake);

    SparkMaxConfig frontRightConfig = new SparkMaxConfig();
    frontRightConfig.follow(Constants.DrivetrainConstants.BackRightId, false);
    frontRightConfig.idleMode(IdleMode.kBrake);

    SparkMaxConfig backRightConfig = new SparkMaxConfig();
    backRightConfig.idleMode(IdleMode.kBrake);

    frontLeft.configure(
        frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backLeft.configure(
        backLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRight.configure(
        frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backRight.configure(
        backRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    differentialDrive = new DifferentialDrive(backLeft, backRight);
  }

  public Command arcadeDrive(DoubleSupplier speed, DoubleSupplier rotation) {
    return run(
        () -> {
          double limitedSpeed = driveLimiter.calculate(speed.getAsDouble());
          double limitedRotation = turnLimiter.calculate(rotation.getAsDouble());

          double forward =
              Math.max(
                  -Constants.DrivetrainConstants.driveMaxPower,
                  Math.min(limitedSpeed, Constants.DrivetrainConstants.driveMaxPower));
          double turn =
              Math.max(
                  -Constants.DrivetrainConstants.turnMaxPower,
                  Math.min(limitedRotation, Constants.DrivetrainConstants.turnMaxPower));
          differentialDrive.arcadeDrive(forward, turn);
        });
  }

  public void resetDriveLimiter() {
    driveLimiter.reset();
  }

  public void resetTurnLimiter() {
    turnLimiter.reset();
  }

  public void resetAccelerationLimiters() {
    resetDriveLimiter();
    resetTurnLimiter();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
