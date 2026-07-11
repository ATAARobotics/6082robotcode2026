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
  private final SparkFlex motor;
  private final SparkClosedLoopController closedLoop;

  private double selectedRpm = 0.0;
  private boolean running = false;

  public Shooter() {
    motor = new SparkFlex(ShooterConstants.motorId, MotorType.kBrushless);
    closedLoop = motor.getClosedLoopController();

    SparkFlexConfig config = new SparkFlexConfig();
    config.inverted(false);
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(ShooterConstants.smartCurrentLimitAmps);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.outputRange(ShooterConstants.minOutput, ShooterConstants.maxOutput);
    config.closedLoop.allowedClosedLoopError(
        ShooterConstants.allowedErrorRpm, ClosedLoopSlot.kSlot0);
    config.closedLoop.pid(ShooterConstants.pidP, ShooterConstants.pidI, ShooterConstants.pidD);
    config.closedLoop.feedForward.sv(ShooterConstants.kS, ShooterConstants.kV);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSelectedRpm(double rpm) {
    selectedRpm = rpm;
    if (running) {
      applySetpoint();
    }
  }

  public void applySelected() {
    running = true;
    applySetpoint();
  }

  public void stop() {
    running = false;
    closedLoop.setSetpoint(0.0, ControlType.kVelocity);
  }

  public double getSelectedRpm() {
    return selectedRpm;
  }

  public double getVelocityRpm() {
    return motor.getEncoder().getVelocity();
  }

  public boolean atSetpoint() {
    return Math.abs(getVelocityRpm() - selectedRpm) <= ShooterConstants.allowedErrorRpm;
  }

  public boolean isRunning() {
    return running;
  }

  private void applySetpoint() {
    closedLoop.setSetpoint(selectedRpm, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Selected RPM", selectedRpm);
    SmartDashboard.putNumber("Shooter/Velocity RPM", getVelocityRpm());
    SmartDashboard.putBoolean("Shooter/At Setpoint", running && atSetpoint());
    SmartDashboard.putBoolean("Shooter/Running", running);
  }

  @Override
  public void simulationPeriodic() {}
}
