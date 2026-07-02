// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
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

  private DifferentialDrive differentialDrive;

  private final Pigeon2 pigeon = new Pigeon2(20);
  private DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(Units.inchesToMeters(24.5));
  private DifferentialDrivePoseEstimator pose =
      new DifferentialDrivePoseEstimator(m_kinematics, pigeon.getRotation2d(), 0, 0, new Pose2d());

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

    SmartDashboard.putData("Field", m_field);
  }

  public Command arcadeDrive(DoubleSupplier x, DoubleSupplier y) {
    return run(
        () -> {
          differentialDrive.arcadeDrive(x.getAsDouble(), y.getAsDouble());
        });
  }

  @Override
  public void periodic() {
    boolean doRejectUpdate = false;

    LimelightHelpers.SetRobotOrientation(
        "limelight", pose.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);

    Optional<Alliance> ally = DriverStation.getAlliance();

    LimelightHelpers.PoseEstimate mt2;
    Pose3d botPose3d;
    if (ally.isPresent() && ally.get() == Alliance.Red) {
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
      botPose3d = LimelightHelpers.getBotPose3d_wpiRed("limelight");
    } else {
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      botPose3d = LimelightHelpers.getBotPose3d_wpiBlue("limelight");
    }

    if (Math.abs(pigeon.getAngularVelocityZDevice().getValueAsDouble()) > 720) {
      doRejectUpdate = true;
    }

    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }

    if (!doRejectUpdate) {
      pose.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      pose.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }

    m_field.setRobotPose(pose.getEstimatedPosition());

    SmartDashboard.putNumber("Drivetrain/X", pose.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Drivetrain/Y", pose.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Drivetrain/Z", botPose3d.getZ());
    SmartDashboard.putNumber("Drivetrain/Yaw", botPose3d.getRotation().getZ());
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
