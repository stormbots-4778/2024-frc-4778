// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverControllerPort2 = 1;
    //button board controller
    public static final int kDriverControllerPort3 = 2;
  }

  public static final class DriveConstants {
    // public static double spdLimitFast = .9; //<======== Competition values
    // public static double turnLimitFast = .9;

    public static double spdLimitFast = .3;
    public static double turnLimitFast = .3;

    public static double spdLimitSlow = .15;
    public static double turnLimitSlow = .15;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.6;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.2; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)
    // private final Swerve swerveSubsystem = new Swerve();

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(20.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs

    // Drive Section
    public static final int kFrontLeftDrivingCanId = 11; //
    public static final int kRearLeftDrivingCanId = 13; //
    public static final int kFrontRightDrivingCanId = 15; //
    public static final int kRearRightDrivingCanId = 17; //

    public static final int kFrontLeftTurningCanId = 10; //
    public static final int kRearLeftTurningCanId = 12; //
    public static final int kFrontRightTurningCanId = 14; //
    public static final int kRearRightTurningCanId = 16; //

    public static final boolean kGyroReversed = false;

    public static final double kDriveBaseRadius = Math.max(kWheelBase, kTrackWidth) / 2.0;
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(6, 0.2, 0.1), // TODO: Tune translation PID config
        new PIDConstants(4, 0.1, 0), // TODO: Tune rotation PID config
        kMaxSpeedMetersPerSecond,
        kDriveBaseRadius,
        new ReplanningConfig());
  }

  public static final class ShooterConstants {
    public static final int kleftShooterCanId = 24; //
    public static final int kRightShooterCanId = 23; //
    public static final int kLauncherPivotCanId = 26;
    public static final double shooterKp = 0.04;
    public static final double shooterKi = 0;
    public static final double shooterKd = 0;

    public static final double leftShootSpeed = 1.0;
    public static final double rightShootSpeed = -1.0;
    public static IdleMode kShootingMotorIdleMode = IdleMode.kBrake;
    public static int kShootingMotorCurrentLimit = 50; // amps

    // Pivot motor configuration

    // TODO: The below values for the launcher pivot are basically random, mean
    // nothing, and HAVE to be tuned

    public static final double kLauncherPivotEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kLauncherPivotEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kLauncherPivotEncoderPositionPIDMinInput = 0; // radians
    public static final double kLauncherPivotEncoderPositionPIDMaxInput = kLauncherPivotEncoderPositionFactor; // radians

    // TODO: Tune PID gains
    public static final double kLauncherPivotP = 0.05;
    public static final double kLauncherPivotI = 0.0001;
    public static final double kLauncherPivotD = 0;
    public static final double kLauncherPivotIZone = 3.0;
    public static final double kLauncherPivotFF = 0;
    public static final double kLauncherPivotMinOutput = -1;
    public static final double kLauncherPivotMaxOutput = 1;

    // TODO: Tune angles for each pivot position
    // public static final double kLauncherPivotAngleIntake = -(Math.PI * 22);
    public static final double kLauncherPivotAngleHigh = 0.0;
    //  public static final double kLauncherPivotAngleHigh = -5.0;
    // public static final double kLauncherPivotAngleLow = -57;
    // public static final double kLauncherPivotAngleLow = -106.875;
    public static final double kLauncherPivotAngleLow = -30;
    public static IdleMode kLauncherPivotMotorIdleMode = IdleMode.kCoast;
    public static final int kLauncherPivotMotorCurrentLimit = 10;

  }

  public static final class LiftConstants {
    public static final int kLiftMotorCanId = 25; //

    public static final double liftKp = 0.04;
    public static final double liftKi = 0;
    public static final double liftKd = 0;

    public static final double liftKFF = 0;
    public static final double kLiftMinOutput = -1;
    public static final double kLiftMaxOutput = 1;

    public static final double kLiftEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kLiftEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kLiftEncoderPositionPIDMinInput = 0; // radians
    public static final double kLiftEncoderPositionPIDMaxInput = kLiftEncoderPositionFactor; // radians

    public static final double kLiftMotorSpeed = 0.5;

   
    public static final double kStepDistance = 20;
    public static final double kFullExtend = 720;
    public static final double kFullRetract = 0;

    public static IdleMode kLiftMotorIdleMode = IdleMode.kBrake;
    public static int kLiftMotorCurrentLimit = 40;
    public static int kLiftMotorZeroingCurrentLimit = 9; // amps
  }

  public static final class IntakeConstants {
    // Intake Motors
    public static final int kBottomRollerCanId = 20; //
    public static final int kTopRollerCanId = 21; //
    public static final int kIntakePivotCanId = 22; //

    public static final double intakeSpeed = 0.75;
    public static final double outtakeSpeed = -1.0;
    public static final double shootSpeedTop = -0.48   ; // top -0.525
    public static double shootSpeedBottom = -0.675; // top -0.73

    public static final double intakeKp = 0.04;
    public static final double intakeKi = 0;
    public static final double intakeKd = 0;

    public static IdleMode kIntakeMotorIdleMode = IdleMode.kCoast;
    public static int kIntakeMotorCurrentLimit = 30; // amps

    // Intake/outtake times
    public static final double outtakeTime = 3.0;

    // Pivot motor configuration
    public static final double kPivotEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kPivotEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kPivotEncoderPositionPIDMinInput = 0; // radians
    public static final double kPivotEncoderPositionPIDMaxInput = kPivotEncoderPositionFactor; // radians

    // TODO: Tune PID gains
    public static final double kPivotP = 0.08;
    public static final double kPivotI = 0.0005;
    public static final double kPivotD = 0;
    public static final double kPivotFF = 0;
    public static final double kPivotIZone = 10.0;
    public static final double kPivotMinOutput = -1;
    public static final double kPivotMaxOutput = 1;

    // TODO: Tune angles for each pivot position
    public static final double kPivotAngleIntake = 72;
    public static final double kPivotAngleAmp = 33;
    // public static final double kPivotAngleSpeaker = 10.0;

    public static final double kPivotAngleSpeaker = 9.0;

    public static final double kPivotAngleLow = 5.5;

    public static final int kPivotMotorCurrentLimit = 35;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    // public static final double kWheelDiameterMeters = 0.072; <==== aluminum wheels
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    // 3/19/2024 Mahi - swerve drive modified to suit 21 teeth
    public static final double kDrivingMotorReduction = (45.0 * 21) / (kDrivingMotorPinionTeeth * 15);

    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.06;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 3;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class LimelightConstants {
    public static final double LimelightMountingAngle = 0; // mounting angle of the camera DEGREES
    public static final double LimelightMountingHeight = 19.5; // height of camera off the ground INCHES (NOT FINAL
                                                               // VALUE WILL KNOW BEFORE LACROSSE)
    public static final double TargetHeight = 41.875; // height of high pole off the ground (Mahi still not figured out
                                                      // how to do multiple targets)

    public static double x; // wrong
    public static double y; // wrong
    public static double area;

    public static boolean limelightToggle = false;

    public static final double AmpTargetTy = 13.0;
    public static final double AmpTargetTx = 0.0;

    public static final double AutoAlignSpeedFactor = 0.1;

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.025;
  }

}