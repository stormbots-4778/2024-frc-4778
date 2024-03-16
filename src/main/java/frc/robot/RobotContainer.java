// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.AutoAim;
import frc.robot.subsystems.LauncherPivotSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
//import frc.robot.commands.CloseShoot;
import frc.robot.commands.SpinLauncher;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;


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
  
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final LiftSubsystem m_lift = new LiftSubsystem();
  public final PivotSubsystem m_pivot = new PivotSubsystem();
  public final IntakeSubsystem m_intake = new IntakeSubsystem(m_launcherSubsystem, m_pivot);
  public final LimelightSubsystem m_limelight = new LimelightSubsystem();
  public final AutoAim m_autoaim = new AutoAim(m_limelight, m_robotDrive, m_intake, m_pivot);

  public boolean Centric = true;
  // public final LauncherPivotSubsystem m_launcherPivot = new LauncherPivotSubsystem();
  //  public final LauncherPivotSubsystem m_launcherPivot = new LauncherPivotSubsystem();


  //public UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
  // private UsbCamera camera;

  // BlinkIn
  // public Spark blinkin = new Spark(1);

  public SendableChooser<Command> autoChooser = new SendableChooser<>();
  public SendableChooser<Command> otherChooser = new SendableChooser<>();

  // Replace with CommandXboxController or CommandJoystick if needed
  public final XboxController m_driverController = 
      new XboxController(OperatorConstants.kDriverControllerPort);
    public final XboxController m_driverController2 = 
      new XboxController(OperatorConstants.kDriverControllerPort2);
    public double spdLimit = DriveConstants.spdLimitFast;
    public double turnLimit = DriveConstants.turnLimitFast;

    public boolean fieldCentric = false;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // camera = CameraServer.startAutomaticCapture();
    // camera.setResolution(80, 60);

    m_robotDrive.setDefaultCommand(
        Commands.run(() -> {
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
            // Rate limit = true sets speed to 0. Why? This is something to fix.
            true, false);
        }, m_robotDrive));

    

    

    

       // NamedCommands.registerCommand("Close Shoot", new CloseShoot(m_launcherSubsystem, m_intake).withTimeout(2));
        NamedCommands.registerCommand("Spin Launcher", new SpinLauncher(m_launcherSubsystem));
        NamedCommands.registerCommand("Shoot", m_launcherSubsystem.shoot());
        NamedCommands.registerCommand("Intake", m_intake.intake());
        NamedCommands.registerCommand("Intake Pivot Position", m_pivot.setPivotGoalCommand(IntakeConstants.kPivotAngleIntake));
        NamedCommands.registerCommand("Amp Position", m_intake.ampPosition());
        NamedCommands.registerCommand("Amp Pivot Position", m_pivot.setPivotGoalCommand(IntakeConstants.kPivotAngleAmp));
        NamedCommands.registerCommand("Turn On Shoot", m_launcherSubsystem.shoot());
        NamedCommands.registerCommand("Turn Off Shoot", m_launcherSubsystem.stop());
        NamedCommands.registerCommand("Speaker Position", m_intake.speakerPosition());
        NamedCommands.registerCommand("Speaker Pivot Position", m_pivot.setPivotGoalCommand(IntakeConstants.kPivotAngleSpeaker));
        NamedCommands.registerCommand("Stop Intake", m_intake.stopIntake());
        NamedCommands.registerCommand("Zero Yaw", m_robotDrive.ZeroHeading());
        // NamedCommands.registerCommand("Speaker Shot", m_launcherPivot.setPivotGoalCommand(IntakeConstants.kPivotAngleSpeaker));
        
        autoChooser = AutoBuilder.buildAutoChooser();
        
        // autoChooser.addOption("MidSpeakerAuto", new MidSpeakerAuto(m_robotDrive, m_intake, m_launcherSubsystem));



        // otherChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // SmartDashboard.putNumber("IMU", m_robotDrive.m_gyro.getAngle());

  }

  // private void toggleFieldCentric(){
  //   fieldCentric = !fieldCentric;
  // }
 


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
            .toggleOnTrue(m_intake.ampPosition())
            .toggleOnTrue(m_pivot.setPivotGoalCommand(IntakeConstants.kPivotAngleAmp))
            .onTrue(m_launcherSubsystem.stop());
   
    new JoystickButton(m_driverController, Button.kY.value)
            .toggleOnTrue(m_intake.intake())
             .toggleOnTrue(m_pivot.setPivotGoalCommand(IntakeConstants.kPivotAngleIntake))
             .onTrue(m_launcherSubsystem.stop())
             
             .onFalse(m_intake.stopIntake());
             
            
    // new JoystickButton(m_driverController.leftTrigger())
    //          .toggleOnTrue(m_intake.intake())
    //          .toggleOnTrue(m_pivot.setPivotGoalCommand(IntakeConstants.kPivotAngleIntake))
    //          .onTrue(m_launcherSubsystem.stop())
             
    //          .onFalse(m_intake.stopIntake());
          
    new JoystickButton(m_driverController, Button.kB.value)
            .toggleOnTrue(m_pivot.setPivotGoalCommand(IntakeConstants.kPivotAngleSpeaker));
    
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
            .toggleOnTrue(m_launcherSubsystem.shoot());

    new JoystickButton(m_driverController, Button.kRightBumper.value)
            .onTrue(m_intake.outtake())
            .onFalse(m_intake.stopIntake());


            
  new JoystickButton(m_driverController, Button.kStart.value)
            .onTrue(m_intake.ampShoot())
            .onTrue(m_launcherSubsystem.stop())
            .onFalse(m_intake.stopIntake());

//   new JoystickButton(m_driverController2, Button.kLeftBumper.value)
//             .toggleOnTrue(m_lift.setLiftGoalCommand(LiftConstants.kFullExtend));
            
//  new JoystickButton(m_driverController2, Button.kRightBumper.value)
//             .toggleOnTrue(m_lift.setLiftGoalCommand(LiftConstants.kFullRetract));

    new JoystickButton(m_driverController2, Button.kLeftBumper.value)
             .toggleOnTrue(m_lift.setLiftGoalCommand(LiftConstants.kFullExtend));
   new JoystickButton(m_driverController2, Button.kRightBumper.value)
             .toggleOnTrue(m_lift.setLiftGoalCommand(LiftConstants.kFullRetract));
    new JoystickButton(m_driverController2, Button.kB.value)
             .whileTrue(m_autoaim.AmpAlign());
  new JoystickButton(m_driverController2, Button.kY.value)
             .onTrue(m_robotDrive.ZeroHeading());

        

            
             
  

            
            
//         new JoystickButton(m_driverController2, Button.kLeftBumper.value)
//              .toggleOnTrue(m_lift.retractStep());
            

  
//  new JoystickButton(m_driverController2, Button.kA.value)
//             .onTrue(m_robotDrive.zeroHeading());
            


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

        // An ExampleCommand will run in autonomous
        // return autoChooser.getSelected();
        return autoChooser.getSelected();

  }

}
