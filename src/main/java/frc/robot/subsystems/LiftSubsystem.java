package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.LiftSubsystem;

public class LiftSubsystem extends TrapezoidProfileSubsystem {
  // private CANSparkMax LiftMotor;
  // private final SparkPIDController LiftPIDController;
  // public static RelativeEncoder LiftEncoder;

  public CANSparkMax LiftMotor;
  public final SparkPIDController LiftPIDController;
  public static RelativeEncoder LiftEncoder;

  private double curGoal = 0.0;

  // public LiftSubsystem() {
  // LiftMotor = new CANSparkMax(LiftConstants.kLiftMotorCanId,
  // MotorType.kBrushless);
  // LiftMotor.restoreFactoryDefaults();
  // LiftMotor.getEncoder().setPosition(0.0);

  // LiftMotor.setIdleMode(LiftConstants.kLiftMotorIdleMode);
  // LiftMotor.setSmartCurrentLimit(LiftConstants.kLiftMotorCurrentLimit);
  // LiftMotor.burnFlash();

  // LiftPIDController = LiftMotor.getPIDController();
  // LiftPIDController.setP(LiftConstants.liftKp);
  // LiftPIDController.setI(LiftConstants.liftKi);
  // LiftPIDController.setD(LiftConstants.liftKd);
  // LiftPIDController.setFF(LiftConstants.liftKFF);

  // LiftPIDController.setOutputRange(LiftConstants.kLiftMinOutput,
  // LiftConstants.kLiftMaxOutput);

  // LiftEncoder = LiftMotor.getEncoder();

  // LiftEncoder.setPositionConversionFactor(LiftConstants.kLiftEncoderPositionFactor);
  // LiftEncoder.setVelocityConversionFactor(LiftConstants.kLiftEncoderVelocityFactor);
  // LiftPIDController.setFeedbackDevice(LiftEncoder);
  // LiftPIDController.setPositionPIDWrappingEnabled(false);
  // LiftPIDController.setPositionPIDWrappingMinInput(LiftConstants.kLiftEncoderPositionPIDMinInput);
  // LiftPIDController.setPositionPIDWrappingMaxInput(LiftConstants.kLiftEncoderPositionPIDMaxInput);

  // }

  // public Command extend() {
  // return runOnce(()-> {
  // LiftMotor.set(LiftConstants.liftMotorSpeed);

  // });
  // }

  // public Command retract() {
  // return runOnce(()-> {
  // LiftMotor.set(-(LiftConstants.liftMotorSpeed));
  // });
  // }

  // public Command stop() {
  // return runOnce(()-> {
  // LiftMotor.set(0.0);

  // });
  // }

  public LiftSubsystem() {

    super(
        new TrapezoidProfile.Constraints(
            450.0, 600.0),
        0.0);

    LiftMotor = new CANSparkMax(LiftConstants.kLiftMotorCanId, MotorType.kBrushless);
    LiftMotor.restoreFactoryDefaults();
    LiftMotor.getEncoder().setPosition(0.0);

    LiftMotor.setIdleMode(LiftConstants.kLiftMotorIdleMode);
    LiftMotor.setSmartCurrentLimit(LiftConstants.kLiftMotorCurrentLimit);
    LiftMotor.burnFlash();

    LiftPIDController = LiftMotor.getPIDController();
    LiftPIDController.setP(LiftConstants.liftKp);
    LiftPIDController.setI(LiftConstants.liftKi);
    LiftPIDController.setD(LiftConstants.liftKd);
    LiftPIDController.setFF(LiftConstants.liftKFF);

    LiftPIDController.setOutputRange(LiftConstants.kLiftMinOutput, LiftConstants.kLiftMaxOutput);

    LiftEncoder = LiftMotor.getEncoder();

    LiftEncoder.setPositionConversionFactor(LiftConstants.kLiftEncoderPositionFactor);
    LiftEncoder.setVelocityConversionFactor(LiftConstants.kLiftEncoderVelocityFactor);
    LiftPIDController.setFeedbackDevice(LiftEncoder);
    LiftPIDController.setPositionPIDWrappingEnabled(false);
    LiftPIDController.setPositionPIDWrappingMinInput(LiftConstants.kLiftEncoderPositionPIDMinInput);
    LiftPIDController.setPositionPIDWrappingMaxInput(LiftConstants.kLiftEncoderPositionPIDMaxInput);

  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    LiftPIDController.setReference(setpoint.position, ControlType.kPosition);
  }

  public Command setLiftGoalCommand(double liftPos) {
    return Commands.runOnce(() -> setGoal(liftPos), this);
  }

  // public Command setLiftGoalCommand(double liftPos) {
  // curGoal = liftPos;
  // return Commands.runOnce(() -> setGoal(curGoal), this);
  // }

  public Command extendStep() {
    return setLiftGoalCommand(Math.min(LiftConstants.kFullExtend, curGoal + LiftConstants.kStepDistance));
  }

  public Command retractStep() {
    return setLiftGoalCommand(Math.max(LiftConstants.kFullRetract, curGoal - LiftConstants.kStepDistance));
  }

}
