// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.AccelerationLimiter;
import frc.robot.util.LimelightHelpers;
import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase {
  private SparkMax frontLeft;
  private SparkMax backLeft;
  private SparkMax frontRight;
  private SparkMax backRight;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private DifferentialDrive differentialDrive;

  private final Pigeon2 pigeon = new Pigeon2(Constants.DrivetrainConstants.PigeonId);
  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(Constants.DrivetrainConstants.trackWidthMeters);
  private final DifferentialDrivePoseEstimator3d poseEstimator =
      new DifferentialDrivePoseEstimator3d(
          m_kinematics, pigeon.getRotation3d(), 0, 0, new Pose3d());

  private final Field2d m_field = new Field2d();
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
    backLeftConfig.encoder.positionConversionFactor(
        Constants.DrivetrainConstants.metersPerRotation);
    backLeftConfig.encoder.velocityConversionFactor(
        Constants.DrivetrainConstants.metersPerRotation / 60.0);

    SparkMaxConfig frontRightConfig = new SparkMaxConfig();
    frontRightConfig.follow(Constants.DrivetrainConstants.BackRightId, false);
    frontRightConfig.idleMode(IdleMode.kBrake);

    SparkMaxConfig backRightConfig = new SparkMaxConfig();
    backRightConfig.idleMode(IdleMode.kBrake);
    backRightConfig.encoder.positionConversionFactor(
        Constants.DrivetrainConstants.metersPerRotation);
    backRightConfig.encoder.velocityConversionFactor(
        Constants.DrivetrainConstants.metersPerRotation / 60.0);

    frontLeft.configure(
        frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backLeft.configure(
        backLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRight.configure(
        frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backRight.configure(
        backRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    differentialDrive = new DifferentialDrive(backLeft, backRight);

    leftEncoder = backLeft.getEncoder();
    rightEncoder = backRight.getEncoder();

    SmartDashboard.putData("Field", m_field);

    LimelightHelpers.setCameraPose_RobotSpace(
        Constants.VisionConstants.limelightName,
        Constants.VisionConstants.cameraForwardMeters,
        Constants.VisionConstants.cameraSideMeters,
        Constants.VisionConstants.cameraUpMeters,
        Constants.VisionConstants.cameraRollDegrees,
        Constants.VisionConstants.cameraPitchDegrees,
        Constants.VisionConstants.cameraYawDegrees);

    LimelightHelpers.SetRobotOrientation(
        Constants.VisionConstants.limelightName,
        poseEstimator.getEstimatedPosition().getRotation().getZ() * (180.0 / Math.PI),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0);
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

  // Yaw component is always 0: pigeon.reset() zeros yaw on hardware, so we only
  // need to offset the unresettable pitch/roll to treat the current pose as (0,0,0).
  private Rotation3d rotationOffset = new Rotation3d();

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition().toPose2d();
  }

  private Rotation3d getAdjustedRotation3d() {
    return pigeon.getRotation3d().minus(rotationOffset);
  }

  public void resetPose(Pose2d pose) {
    pigeon.reset();
    rotationOffset = pigeon.getRotation3d();
    poseEstimator.resetPosition(getAdjustedRotation3d(), 0, 0, new Pose3d(pose));
  }

  public void zeroHeading() {
    pigeon.reset();
    rotationOffset = pigeon.getRotation3d();
    poseEstimator.resetRotation(new Rotation3d());
  }

  private boolean rejectsNoTags(LimelightHelpers.PoseEstimate estimate) {
    return estimate.tagCount == 0;
  }

  private boolean rejectsSpinningTooFast(LimelightHelpers.PoseEstimate estimate) {
    return Math.abs(pigeon.getAngularVelocityZDevice().getValueAsDouble()) > 720;
  }

  private boolean shouldRejectVisionUpdate(LimelightHelpers.PoseEstimate estimate) {
    return rejectsNoTags(estimate) || rejectsSpinningTooFast(estimate);
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
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.VisionConstants.limelightName);

    poseEstimator.update(
        getAdjustedRotation3d(), leftEncoder.getPosition(), -rightEncoder.getPosition());

    if (!shouldRejectVisionUpdate(mt2)) {
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.2, 0.2, 9999999, 9999999));
      poseEstimator.addVisionMeasurement(new Pose3d(mt2.pose), mt2.timestampSeconds);
    }

    Pose3d currentPose = poseEstimator.getEstimatedPosition();

    m_field.setRobotPose(currentPose.toPose2d());

    SmartDashboard.putNumber("Drivetrain/X", currentPose.getX());
    SmartDashboard.putNumber("Drivetrain/Y", currentPose.getY());
    SmartDashboard.putNumber("Drivetrain/Z", currentPose.getZ());
    SmartDashboard.putNumber("Drivetrain/Yaw", currentPose.getRotation().getZ());
    SmartDashboard.putNumber("Drivetrain/Pitch", pigeon.getPitch().getValueAsDouble());
    SmartDashboard.putNumber("Drivetrain/Roll", pigeon.getRoll().getValueAsDouble());
    SmartDashboard.putNumber(
        "Drivetrain/Velocity", pigeon.getAngularVelocityZDevice().getValueAsDouble());
    SmartDashboard.putNumber("Drivetrain/LeftBackEncoder", leftEncoder.getPosition());
    SmartDashboard.putNumber("Drivetrain/RightBackEncoder", -rightEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
