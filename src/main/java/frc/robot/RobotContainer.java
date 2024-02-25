// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem();
  public final IntakeSubsystem m_intake = new IntakeSubsystem(m_launcherSubsystem);
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // Replace with CommandXboxController or CommandJoystick if needed
  public final XboxController m_driverController = 
      new XboxController(OperatorConstants.kDriverControllerPort);
    public double spdLimit = DriveConstants.spdLimitFast;
    public double turnLimit = DriveConstants.turnLimitFast;

    public boolean fieldCentric;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_robotDrive.setDefaultCommand(
        Commands.run(() -> 
          m_robotDrive.drive(
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
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandXboxController
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));
    
    // new JoystickButton(m_driverController, Button.kLeftBumper.value)
    //         .whileTrue(new InstantCommand(
    //         ()->m_intake.stowPos(),
    //         m_intake));     

    // new JoystickButton(m_driverController, Button.kRightBumper.value)
    //         .whileTrue(new InstantCommand(
    //         ()->m_intake.deployPos(),
    //         m_intake));

    new JoystickButton(m_driverController, Button.kX.value)
            .onTrue(m_intake.ampPosition());
            
    new JoystickButton(m_driverController, Button.kY.value)
             .onTrue(m_intake.intake())
             .onFalse(m_intake.stopIntake());
          
    new JoystickButton(m_driverController, Button.kB.value)
            .onTrue(m_intake.speakerPosition());

    new JoystickButton(m_driverController, Button.kA.value)
            .onTrue(m_intake.outtake())
            .onFalse(m_intake.stopIntake());

new JoystickButton(m_driverController, Button.kRightBumper.value)
            .onTrue(m_launcherSubsystem.shoot())
            .onFalse(m_launcherSubsystem.stop());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.duluthAuto(m_robotDrive, m_intake);
  }
}
