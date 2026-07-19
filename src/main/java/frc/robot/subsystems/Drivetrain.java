// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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

  private final Field2d field = new Field2d();
  private AccelerationLimiter driveLimiter =
      new AccelerationLimiter(
          Constants.DrivetrainConstants.driveMaxRatePerSec,
          Constants.DrivetrainConstants.driveCurveExponent);
  private AccelerationLimiter turnLimiter =
      new AccelerationLimiter(
          Constants.DrivetrainConstants.turnMaxRatePerSec,
          Constants.DrivetrainConstants.turnCurveExponent);

  private SparkClosedLoopController leftClosedLoopController;
  private SparkClosedLoopController rightClosedLoopController;

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
    backLeftConfig.closedLoop.pid(
        Constants.DrivetrainConstants.leftP,
        Constants.DrivetrainConstants.leftI,
        Constants.DrivetrainConstants.leftD);

    SparkMaxConfig frontRightConfig = new SparkMaxConfig();
    frontRightConfig.follow(Constants.DrivetrainConstants.BackRightId, false);
    frontRightConfig.idleMode(IdleMode.kBrake);

    SparkMaxConfig backRightConfig = new SparkMaxConfig();
    backRightConfig.idleMode(IdleMode.kBrake);
    backRightConfig.inverted(true);
    backRightConfig.encoder.positionConversionFactor(
        Constants.DrivetrainConstants.metersPerRotation);
    backRightConfig.encoder.velocityConversionFactor(
        Constants.DrivetrainConstants.metersPerRotation / 60.0);
    backRightConfig.closedLoop.pid(
        Constants.DrivetrainConstants.rightP,
        Constants.DrivetrainConstants.rightI,
        Constants.DrivetrainConstants.rightD);

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

    leftClosedLoopController = backLeft.getClosedLoopController();
    rightClosedLoopController = backRight.getClosedLoopController();

    try {
      Constants.robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            driveRobotRelative(
                speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        new PPLTVController(
            0.02), // PPLTVController is the built in path following controller for differential
        // drive trains
        Constants.robotConfig, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );

    SmartDashboard.putData("Field", field);

    SmartDashboard.putNumber("LeftPID/Left P", Constants.DrivetrainConstants.leftP);
    SmartDashboard.putNumber("LeftPID/Left I", Constants.DrivetrainConstants.leftI);
    SmartDashboard.putNumber("LeftPID/Left D", Constants.DrivetrainConstants.leftD);

    SmartDashboard.putNumber("RightPID/Right P", Constants.DrivetrainConstants.rightP);
    SmartDashboard.putNumber("RightPID/Right I", Constants.DrivetrainConstants.rightI);
    SmartDashboard.putNumber("RightPID/Right D", Constants.DrivetrainConstants.rightD);
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

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition().toPose2d();
  }

  public void resetPose(Pose2d pose) {
    pigeon.setYaw(pose.getRotation().getDegrees());
    poseEstimator.resetPosition(pigeon.getRotation3d(), 0, 0, new Pose3d(pose));
  }

  public void zeroHeading() {
    pigeon.reset();
    poseEstimator.resetRotation(new Rotation3d());
  }

  private boolean rejectsNoTags(LimelightHelpers.PoseEstimate estimate) {
    return estimate.tagCount == 0;
  }

  private boolean rejectsSpinningTooFast() {
    return Math.abs(pigeon.getAngularVelocityZDevice().getValueAsDouble()) > 720;
  }

  private boolean shouldRejectVisionUpdate(LimelightHelpers.PoseEstimate estimate) {
    return rejectsNoTags(estimate) || rejectsSpinningTooFast();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity()));
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(Constants.DrivetrainConstants.maxDriveSpeed);

    SmartDashboard.putNumber("Left Motor Speed (m/s)", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Motor Speed (m/s)", wheelSpeeds.rightMetersPerSecond);

    leftClosedLoopController.setSetpoint(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity);
    rightClosedLoopController.setSetpoint(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity);
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
        pigeon.getRotation3d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    if (!shouldRejectVisionUpdate(mt2)) {
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.2, 0.2, 9999999, 9999999));
      poseEstimator.addVisionMeasurement(new Pose3d(mt2.pose), mt2.timestampSeconds);
    }

    Pose3d currentPose = poseEstimator.getEstimatedPosition();

    field.setRobotPose(currentPose.toPose2d());

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          field.getObject("target pose").setPose(pose);
        });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          // Do whatever you want with the poses here
          field.getObject("path").setPoses(poses);
        });

    SparkMaxConfig backLeftConfig = new SparkMaxConfig();
    SparkMaxConfig backRightConfig = new SparkMaxConfig();

    backLeftConfig.closedLoop.pid(
        SmartDashboard.getNumber("LeftPID/Left P", Constants.DrivetrainConstants.leftP),
        SmartDashboard.getNumber("LeftPID/Left I", Constants.DrivetrainConstants.leftI),
        SmartDashboard.getNumber("LeftPID/Left D", Constants.DrivetrainConstants.leftD));

    backRightConfig.closedLoop.pid(
        SmartDashboard.getNumber("RightPID/Right P", Constants.DrivetrainConstants.rightP),
        SmartDashboard.getNumber("RightPID/Right I", Constants.DrivetrainConstants.rightI),
        SmartDashboard.getNumber("RightPID/Right D", Constants.DrivetrainConstants.rightD));

    backLeft.configure(
        backLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    backRight.configure(
        backRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SmartDashboard.putData("Field", field);
    SmartDashboard.putNumber("Drivetrain/X", currentPose.getX());
    SmartDashboard.putNumber("Drivetrain/Y", currentPose.getY());
    SmartDashboard.putNumber("Drivetrain/Z", currentPose.getZ());
    SmartDashboard.putNumber("Drivetrain/Yaw", currentPose.getRotation().getZ());
    SmartDashboard.putNumber("Drivetrain/Pitch", pigeon.getPitch().getValueAsDouble());
    SmartDashboard.putNumber("Drivetrain/Roll", pigeon.getRoll().getValueAsDouble());
    SmartDashboard.putNumber(
        "Drivetrain/Velocity", pigeon.getAngularVelocityZDevice().getValueAsDouble());
    SmartDashboard.putNumber("Drivetrain/LeftBackEncoder", leftEncoder.getPosition());
    SmartDashboard.putNumber("Drivetrain/RightBackEncoder", rightEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
