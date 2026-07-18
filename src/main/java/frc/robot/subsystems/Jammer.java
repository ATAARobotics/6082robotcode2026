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

// import frc.robot.Constants.JammerConstants;

/** Subsystem that runs the Jammer roller at a constant speed while commanded. */
public class Jammer extends SubsystemBase {
  private final SparkFlex motor;

  public Jammer() {
    motor =
        new SparkFlex(
            Constants.ShooterConstants.JammerConstants.jammerMotorId, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();
    config.inverted(true);
    config.idleMode(IdleMode.kBrake);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command runJammer(double speed) {
    return run(() -> motor.set(speed));
  }
}
