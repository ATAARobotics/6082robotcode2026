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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and command mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.driverControllerPort);

  private final SendableChooser<Integer> m_startSlotChooser = new SendableChooser<>();

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
            .arcadeDrive(m_driverController::getLeftY, m_driverController::getRightX)
            .beforeStarting(drivetrain::resetAccelerationLimiters));
  }

  public Pose2d getStartPose() {
    Integer slot = m_startSlotChooser.getSelected();
    int slotIndex = (slot == null) ? 1 : slot;
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Red) {
      return switch (slotIndex) {
        case 2 -> Constants.StartPoses.kRedStart2;
        case 3 -> Constants.StartPoses.kRedStart3;
        default -> Constants.StartPoses.kRedStart1;
      };
    }
    return switch (slotIndex) {
      case 2 -> Constants.StartPoses.kBlueStart2;
      case 3 -> Constants.StartPoses.kBlueStart3;
      default -> Constants.StartPoses.kBlueStart1;
    };
  }

  public void resetStartPose() {
    drivetrain.resetPose(getStartPose());
  }

  public void zeroHeading() {
    drivetrain.zeroHeading();
  }

  public Command getAutonomousCommand() {
    return Autos.exampleAuto(drivetrain);
  }
}
