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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase {
  private SparkMax frontLeft;
  private SparkMax backLeft;
  private SparkMax frontRight;
  private SparkMax backRight;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private DifferentialDrive differentialDrive;

  private final Pigeon2 pigeon = new Pigeon2(20);
  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(Constants.DrivetrainConstants.kTrackWidthMeters);
  private final DifferentialDrivePoseEstimator3d poseEstimator =
      new DifferentialDrivePoseEstimator3d(
          m_kinematics, pigeon.getRotation3d(), 0, 0, new Pose3d());

  private final Field2d m_field = new Field2d();

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
        Constants.DrivetrainConstants.kMetersPerRotation);
    backLeftConfig.encoder.velocityConversionFactor(
        Constants.DrivetrainConstants.kMetersPerRotation / 60.0);

    SparkMaxConfig frontRightConfig = new SparkMaxConfig();
    frontRightConfig.follow(Constants.DrivetrainConstants.BackRightId, false);
    frontRightConfig.idleMode(IdleMode.kBrake);

    SparkMaxConfig backRightConfig = new SparkMaxConfig();
    backRightConfig.idleMode(IdleMode.kBrake);
    backRightConfig.encoder.positionConversionFactor(
        Constants.DrivetrainConstants.kMetersPerRotation);
    backRightConfig.encoder.velocityConversionFactor(
        Constants.DrivetrainConstants.kMetersPerRotation / 60.0);

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
        Constants.VisionConstants.kLimelightName,
        Constants.VisionConstants.kCameraForwardMeters,
        Constants.VisionConstants.kCameraSideMeters,
        Constants.VisionConstants.kCameraUpMeters,
        Constants.VisionConstants.kCameraRollDegrees,
        Constants.VisionConstants.kCameraPitchDegrees,
        Constants.VisionConstants.kCameraYawDegrees);
  }

  public Command arcadeDrive(DoubleSupplier x, DoubleSupplier y) {
    return run(
        () -> {
          differentialDrive.arcadeDrive(x.getAsDouble(), y.getAsDouble());
        });
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition().toPose2d();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(pigeon.getRotation3d(), 0, 0, new Pose3d(pose));
  }

  public void zeroHeading() {
    pigeon.setYaw(0);
    poseEstimator.resetRotation(new Rotation3d());
  }

  @Override
  public void periodic() {
    boolean doRejectUpdate = false;

    LimelightHelpers.SetRobotOrientation(
        "limelight",
        poseEstimator.getEstimatedPosition().getRotation().getZ() * (180.0 / Math.PI),
        pigeon.getAngularVelocityZDevice().getValueAsDouble(),
        pigeon.getPitch().getValueAsDouble(),
        pigeon.getRoll().getValueAsDouble(),
        pigeon.getAngularVelocityYDevice().getValueAsDouble(),
        pigeon.getAngularVelocityXDevice().getValueAsDouble());

    Optional<Alliance> ally = DriverStation.getAlliance();

    LimelightHelpers.PoseEstimate mt2;
    if (ally.isPresent() && ally.get() == Alliance.Red) {
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
    } else {
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    }

    if (Math.abs(pigeon.getAngularVelocityZDevice().getValueAsDouble()) > 720) {
      doRejectUpdate = true;
    }

    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }

    poseEstimator.update(
        pigeon.getRotation3d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    if (!doRejectUpdate) {
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
