package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final SparkFlex shooterMotor;
  private final SparkFlex indexMotor;
  private final SparkClosedLoopController shooterClosedLoop;
  private final SparkClosedLoopController indexClosedLoop;

  private double selectedShooterRpm = 0.0;
  private double selectedIndexRpm = 0.0;
  private boolean runningShooter = false;
  private boolean runningIndex = false;

  public Shooter() {
    // Shooter
    shooterMotor = new SparkFlex(ShooterConstants.Shooter.motorId, MotorType.kBrushless);
    shooterClosedLoop = shooterMotor.getClosedLoopController();

    // TODO: CLEANUP also it still runs at setpoint 0

    SparkFlexConfig shooterConfig = new SparkFlexConfig();
    shooterConfig.inverted(true);
    shooterConfig.idleMode(IdleMode.kCoast);
    // shooterConfig.smartCurrentLimit(ShooterConstants.Shooter.smartCurrentLimitAmps);

    // shooterConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // shooterConfig.closedLoop.outputRange(ShooterConstants.Shooter.minOutput, ShooterConstants.Shooter.maxOutput);
    // shooterConfig.closedLoop.allowedClosedLoopError(
    //     ShooterConstants.Shooter.allowedErrorRpm, ClosedLoopSlot.kSlot0);
    shooterConfig.closedLoop.pid(ShooterConstants.Shooter.pidP, ShooterConstants.Shooter.pidI, ShooterConstants.Shooter.pidD);
    shooterConfig.closedLoop.feedForward.sv(ShooterConstants.Shooter.kS, ShooterConstants.Shooter.kV);

    shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Index
    indexMotor = new SparkFlex(ShooterConstants.Index.motorId, MotorType.kBrushless);
    indexClosedLoop = indexMotor.getClosedLoopController();

    SparkFlexConfig indexConfig = new SparkFlexConfig();
    indexConfig.inverted(true);
    indexConfig.idleMode(IdleMode.kCoast);
    indexConfig.encoder.positionConversionFactor(0.7058823529411765);
    // indexConfig.smartCurrentLimit(ShooterConstants.Index.smartCurrentLimitAmps);

    // indexConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // indexConfig.closedLoop.outputRange(ShooterConstants.Index.minOutput, ShooterConstants.Index.maxOutput);
    // indexConfig.closedLoop.allowedClosedLoopError(
    //     ShooterConstants.Index.allowedErrorRpm, ClosedLoopSlot.kSlot0);
    indexConfig.closedLoop.pid(ShooterConstants.Index.pidP, ShooterConstants.Index.pidI, ShooterConstants.Index.pidD);
    indexConfig.closedLoop.feedForward.sv(ShooterConstants.Index.kS, ShooterConstants.Index.kV);

    indexMotor.configure(indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Shooter/Shooter/Setpoint Override", 0.0);
    SmartDashboard.putNumber("Shooter/Index/Setpoint Override", 0.0);
  }

  public void setSelectedShooterRpm(double rpm) {
    selectedShooterRpm = rpm;
    if (runningShooter) {
      applyShooterSetpoint();
    }
  }

  public void setSelectedIndexRpm(double rpm) {
    selectedIndexRpm = rpm;
    if (runningIndex) {
      applyIndexSetpoint();
    }
  }

  public void applySelectedShooter() {
    runningShooter = true;
    applyShooterSetpoint();
  }

  public void applySelectedIndex() {
    runningIndex = true;
    applyIndexSetpoint();
  }

  public void applySelected() {
    applySelectedShooter();
    applySelectedIndex();
  }

  public void applyShooterOverride(double rpm) {
    shooterClosedLoop.setSetpoint(rpm, ControlType.kVelocity);
  }
  
  public void applyIndexOverride(double rpm) {
    indexClosedLoop.setSetpoint(rpm, ControlType.kVelocity);
  }

  public void stopShooter() {
    runningShooter = false;
    shooterClosedLoop.setSetpoint(0.0, ControlType.kVelocity);
  }

  public void stopIndex() {
    runningIndex = false;
    indexClosedLoop.setSetpoint(0.0, ControlType.kVelocity);
  }

  public void stop() {
    stopShooter();
    stopIndex();
  }
 
  public double getSelectedShooterRpm() {
    return selectedShooterRpm;
  }

  public double getShooterVelocityRpm() {
    return shooterMotor.getEncoder().getVelocity();
  }

  public double getIndexVelocityRpm() {
    return indexMotor.getEncoder().getVelocity();
  }

  public boolean shooterAtSetpoint() {
    return Math.abs(getShooterVelocityRpm() - selectedShooterRpm) <= ShooterConstants.Shooter.allowedErrorRpm;
  }

  public boolean indexAtSetpoint() {
    return Math.abs(getIndexVelocityRpm() - selectedIndexRpm) <= ShooterConstants.Index.allowedErrorRpm;
  }

  public boolean isShooterRunning() {
    return runningShooter;
  }

  public boolean isIndexRunning() {
    return runningIndex;
  }

  private void applyShooterSetpoint() {
    shooterClosedLoop.setSetpoint(selectedShooterRpm, ControlType.kVelocity);
  }

  private void applyIndexSetpoint() {
    indexClosedLoop.setSetpoint(selectedIndexRpm, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    double overrideShooterRpm = SmartDashboard.getNumber("Shooter/Shooter/Setpoint Override", 0.0);
    if (runningShooter && overrideShooterRpm != 0.0) {
      selectedShooterRpm = overrideShooterRpm;
      shooterClosedLoop.setSetpoint(overrideShooterRpm, ControlType.kVelocity);
    }

    double overrideIndexRpm = SmartDashboard.getNumber("Shooter/Index/Setpoint Override", 0.0);
    if (runningIndex && overrideIndexRpm != 0.0) {
      selectedIndexRpm = overrideIndexRpm;
      indexClosedLoop.setSetpoint(overrideIndexRpm, ControlType.kVelocity);
    }

    SmartDashboard.putNumber("Shooter/Shooter/Selected RPM", selectedShooterRpm);
    SmartDashboard.putNumber("Shooter/Shooter/Velocity RPM", getShooterVelocityRpm());
    SmartDashboard.putBoolean("Shooter/Shooter/At Setpoint", runningShooter && shooterAtSetpoint());
    SmartDashboard.putBoolean("Shooter/Shooter/Running", runningShooter);

    SmartDashboard.putNumber("Shooter/Index/Selected RPM", selectedIndexRpm);
    SmartDashboard.putNumber("Shooter/Index/Velocity RPM", getIndexVelocityRpm());
    SmartDashboard.putBoolean("Shooter/Index/At Setpoint", runningIndex && indexAtSetpoint());
    SmartDashboard.putBoolean("Shooter/Index/Running", runningIndex);
  }

  @Override
  public void simulationPeriodic() {}
}
