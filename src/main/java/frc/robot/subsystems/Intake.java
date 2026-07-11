package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Subsystem that runs the intake roller at a constant speed while commanded. */
public class Intake extends SubsystemBase {
  private final SparkFlex motor;

  public Intake() {
    motor = new SparkFlex(Constants.IntakeConstants.intakeMotorId, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command runIntake() {
    return runEnd(() -> motor.set(Constants.IntakeConstants.intakeSpeed), () -> motor.set(0));
  }
}
