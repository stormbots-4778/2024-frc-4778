// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.CloseShoot;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem();
  public final IntakeSubsystem m_intake = new IntakeSubsystem(m_launcherSubsystem);
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final LiftSubsystem m_lift = new LiftSubsystem();

  public final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);

  public final SendableChooser<Command> autoChooser;
  public double spdLimit = DriveConstants.spdLimitFast;
  public double turnLimit = DriveConstants.turnLimitFast;
  public boolean fieldCentric;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_robotDrive.setDefaultCommand(
        Commands.run(() -> m_robotDrive.drive(
            -MathUtil.applyDeadband(
                Math.pow(m_driverController.getLeftY(), 2) * Math.signum(m_driverController.getLeftY()) * spdLimit,
                OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(
                Math.pow(m_driverController.getLeftX(), 2) * Math.signum(m_driverController.getLeftX()) * spdLimit,
                OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(
                Math.pow(m_driverController.getRightX(), 2) * Math.signum(m_driverController.getRightX()) * turnLimit,
                OIConstants.kDriveDeadband),
            fieldCentric, true),
            m_robotDrive));

    NamedCommands.registerCommand("Close Shoot", new CloseShoot(m_launcherSubsystem, m_intake).withTimeout(2));
    NamedCommands.registerCommand("Intake", m_intake.intake());
    NamedCommands.registerCommand("Amp Position", m_intake.ampPosition());
    NamedCommands.registerCommand("Speaker Position", m_intake.speakerPosition());
    NamedCommands.registerCommand("Stop Intake", m_intake.stopIntake());

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("DuluthSimpleAuto", Autos.duluthAuto(m_robotDrive, m_intake, m_launcherSubsystem));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandXboxController
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // new JoystickButton(m_driverController, Button.kLeftBumper.value)
    // .whileTrue(new InstantCommand(
    // ()->m_intake.stowPos(),
    // m_intake));

    // new JoystickButton(m_driverController, Button.kRightBumper.value)
    // .whileTrue(new InstantCommand(
    // ()->m_intake.deployPos(),
    // m_intake));

    new JoystickButton(m_driverController, Button.kX.value)
        .onTrue(m_intake.ampPosition());

    new JoystickButton(m_driverController, Button.kY.value)
        .onTrue(m_intake.intake())
        .onFalse(m_intake.stopIntake());

    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(m_launcherSubsystem.shoot()),
        m_launcherSubsystem.waitSeconds(2.0),
        .onTrue(m_intake.speakerPosition());
        

    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(m_intake.outtake())
        .onFalse(m_intake.stopIntake());

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .onTrue(m_lift.extend())
        .onFalse(m_lift.stop());

    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(m_lift.retract())
        .onFalse(m_lift.stop());

    // new JoystickButton(m_driverController, Button.kRightBumper.value)
    // .onTrue(m_launcherSubsystem.shoot())
    // .onFalse(m_launcherSubsystem.stop());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
