// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  // TODO: This might be useful as a starting point for starting to build up an auto routine
  public static Command duluthAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, LauncherSubsystem m_launcherSubsystem) {
    return Commands.sequence(
      // 1. We start with a note loaded and in shooting position.. shoot it?
      Commands.print("Starting auto routine"),
      Commands.print("Confirm speaker position"),
      intakeSubsystem.speakerPosition(),  // Confirm we are in shooting position and start the launcher motors
      Commands.waitSeconds(1.0),  // Wait for launcher to spin up
      Commands.print("shoot!"),
      intakeSubsystem.outtake(),          // Shoot!
      Commands.waitSeconds(2.0),  // Wait for note to fire

      // 2. Lower to intake position and start the intake motors
      Commands.print("Intake position"),
      intakeSubsystem.intake(),
      intakeSubsystem.stopIntake(), // TODO: Need to detect if a note has been picked up and automatically stop the intake motors

      // 3. Go somewhere?
      Commands.print("Let's drive"),
      Commands.runOnce(() -> driveSubsystem.drive(0.5, 0.5, 0.0, false, false), driveSubsystem),
      Commands.waitSeconds(3.0),
      Commands.runOnce(() -> driveSubsystem.stopModules(), driveSubsystem),
      Commands.print("Stopped")
    );  
  }

  public static Command testAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    PathPlannerPath start = PathPlannerPath.fromPathFile("Start");

    return Commands.sequence(
      AutoBuilder.followPath(start)
    );
  }

  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
