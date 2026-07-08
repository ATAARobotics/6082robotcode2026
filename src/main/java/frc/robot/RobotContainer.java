// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Jammer;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Jammer jammer = new Jammer();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.driverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.operatorControllerPort);

  private final SendableChooser<Integer> m_startSlotChooser = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser;

  private boolean indexLatched = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_startSlotChooser.setDefaultOption("Slot 1", 1);
    m_startSlotChooser.addOption("Slot 2", 2);
    m_startSlotChooser.addOption("Slot 3", 3);
    SmartDashboard.putData("Start Slot", m_startSlotChooser);

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain
            .arcadeDrive(() -> -m_driverController.getLeftY(), () -> m_driverController.getRightX())
            .beforeStarting(drivetrain::resetAccelerationLimiters));

    m_operatorController
        .leftTrigger()
        .onTrue(intake.runIntake(Constants.IntakeConstants.intakeSpeed))
        .onFalse(intake.runIntake(0));
    // m_operatorController.rightTrigger().onTrue(shooter.runShooter(0.6)).onFalse(shooter.runShooter(0));
    // m_operatorController.b().onTrue(shooter.runIndexerCommand(0.5)).onFalse(shooter.runIndexerCommand(0));
    m_operatorController
        .rightTrigger()
        .onTrue(new InstantCommand(() -> shooter.applyIndexOverride(2000)))
        .onFalse(new InstantCommand(() -> shooter.applyIndexOverride(-800)));
    m_operatorController.rightBumper().onTrue(jammer.runJammer(0.4)).onFalse(jammer.runJammer(0.0));
    m_operatorController.leftBumper().onTrue(jammer.runJammer(-0.8)).onFalse(jammer.runJammer(0.0));
    m_operatorController
        .povLeft()
        .onTrue(new InstantCommand(() -> shooter.applyShooterOverride(3660)));
    m_operatorController
        .povRight()
        .onTrue(new InstantCommand(() -> shooter.applyShooterOverride(4500)));
    m_operatorController
        .povUp()
        .onTrue(new InstantCommand(() -> shooter.applyShooterOverride(4050)));
    m_operatorController
        .povDown()
        .onTrue(new InstantCommand(() -> shooter.applyShooterOverride(1_000_000)));
    m_operatorController.y().onTrue(new InstantCommand(() -> shooter.stopShooter()));

    shooter.applyIndexOverride(-800);
    shooter.applyShooterOverride(4050);
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
    return autoChooser.getSelected();
  }
}
