// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();
  private final Shooter shooter = new Shooter();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.driverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.operatorControllerPort);

  private final SendableChooser<Integer> m_startSlotChooser = new SendableChooser<>();

  private boolean indexLatched = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_startSlotChooser.setDefaultOption("Slot 1", 1);
    m_startSlotChooser.addOption("Slot 2", 2);
    m_startSlotChooser.addOption("Slot 3", 3);
    SmartDashboard.putData("Start Slot", m_startSlotChooser);

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain
            .arcadeDrive(() -> -m_driverController.getLeftY(), () -> m_driverController.getRightX())
            .beforeStarting(drivetrain::resetAccelerationLimiters));

    m_operatorController
        .a()
        .onTrue(shooter.runOnce(() -> {
          shooter.setSelectedShooterRpm(ShooterConstants.Shooter.setpointLowRpm);
          shooter.setSelectedIndexRpm(ShooterConstants.Index.setpointLowRpm);
        }));
    m_operatorController
        .b()
        .onTrue(shooter.runOnce(() -> {
          shooter.setSelectedShooterRpm(ShooterConstants.Shooter.setpointMidRpm);
          shooter.setSelectedIndexRpm(ShooterConstants.Index.setpointMidRpm);
        }));
    m_operatorController
        .x()
        .onTrue(shooter.runOnce(() -> {
          shooter.setSelectedShooterRpm(ShooterConstants.Shooter.setpointHighRpm);
          shooter.setSelectedIndexRpm(ShooterConstants.Index.setpointHighRpm);
        }));

    BooleanSupplier triggerHeld = m_operatorController.rightTrigger();
    BooleanSupplier overrideHeld = m_operatorController.povUp();

    Trigger indexShouldRun = new Trigger(() -> {
      boolean trigger = triggerHeld.getAsBoolean();
      boolean override = overrideHeld.getAsBoolean();
      boolean ready = shooter.indexReadyToSpin();

      if (!trigger) {
        indexLatched = false;
      } else if (ready) {
        indexLatched = true;
      }
      return override || (trigger && indexLatched);
    });

    Command spinUpShooter = Commands.startEnd(
        shooter::applySelectedShooter, shooter::stopShooter, shooter);
    Command runIndex = Commands.startEnd(
        shooter::applySelectedIndex, shooter::stopIndex, shooter);

    new Trigger(triggerHeld).whileTrue(spinUpShooter);
    indexShouldRun.whileTrue(runIndex);
  }

  public Pose2d getStartPose() {
    Integer slot = m_startSlotChooser.getSelected();
    int slotIndex = (slot == null) ? 1 : slot;
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Red) {
      return switch (slotIndex) {
        case 2 -> Constants.StartPoses.redStart2;
        case 3 -> Constants.StartPoses.redStart3;
        default -> Constants.StartPoses.redStart1;
      };
    }
    return switch (slotIndex) {
      case 2 -> Constants.StartPoses.blueStart2;
      case 3 -> Constants.StartPoses.blueStart3;
      default -> Constants.StartPoses.blueStart1;
    };
  }

  public void resetStartPose() {
    drivetrain.resetPose(getStartPose());
  }

  public Command getAutonomousCommand() {
    return Autos.exampleAuto(drivetrain);
  }
}
