package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class PivotSubsystem extends TrapezoidProfileSubsystem {
  private final CANSparkMax pivotMotor = new CANSparkMax(IntakeConstants.kIntakePivotCanId, MotorType.kBrushless);
  private final SparkPIDController pivotPIDController;
  public static RelativeEncoder pivotEncoder;
  public boolean inSpeakerPosition = true;
  public boolean isResetting = false;

  public PivotSubsystem() {

    super(
        new TrapezoidProfile.Constraints(
            200.0, 200.0),    // 200, 300
        0.0);

    pivotMotor.restoreFactoryDefaults();
    pivotMotor.getEncoder().setPosition(0.0);

    pivotMotor.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
    pivotMotor.setSmartCurrentLimit(IntakeConstants.kPivotMotorCurrentLimit);

    pivotEncoder = pivotMotor.getEncoder();

    pivotEncoder.setPositionConversionFactor(IntakeConstants.kPivotEncoderPositionFactor);
    pivotEncoder.setVelocityConversionFactor(IntakeConstants.kPivotEncoderVelocityFactor);

    pivotPIDController = pivotMotor.getPIDController();
    pivotPIDController.setFeedbackDevice(pivotEncoder);
    pivotPIDController.setPositionPIDWrappingEnabled(false);
    pivotPIDController.setPositionPIDWrappingMinInput(IntakeConstants.kPivotEncoderPositionPIDMinInput);
    pivotPIDController.setPositionPIDWrappingMaxInput(IntakeConstants.kPivotEncoderPositionPIDMaxInput);
    pivotPIDController.setP(IntakeConstants.kPivotP);
    pivotPIDController.setI(IntakeConstants.kPivotI);
    pivotPIDController.setD(IntakeConstants.kPivotD);
    pivotPIDController.setFF(IntakeConstants.kPivotFF);
    pivotPIDController.setIZone(IntakeConstants.kPivotIZone);
    pivotMotor.burnFlash();

  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    pivotPIDController.setReference(setpoint.position, ControlType.kPosition);
  }

  public Command setPivotGoalCommand(double pivotPos) {
    return Commands.runOnce(() -> setGoal(pivotPos), this);
  }

  public Command speakerPosition(){
    return runOnce(() -> {
      this.S();
    });
  }

  public Command notSpeakerPosition(){
    return runOnce(() -> {
      this.nS();
    });
  }

  public void S(){
    inSpeakerPosition = true;
    System.out.println("Speaker Position");
  }

  public void nS(){
    inSpeakerPosition = false;
    System.out.println("Not Speaker Position");
  }

  public void intakePos(double pivotPos) {
    this.setGoal(pivotPos);
  }
  

  public Command resetAxis() {
    return run(() -> {
      this.isResetting = true;
      pivotMotor.setVoltage(-0.2);
      pivotPIDController.setP(0);
      pivotPIDController.setI(0);
      pivotPIDController.setD(0);
    });
  }

  public Command resetEncoder() {
    return runOnce(() -> {
      pivotEncoder.setPosition(0.0);
      this.isResetting = false;
      pivotPIDController.setP(IntakeConstants.kPivotP);
      pivotPIDController.setI(IntakeConstants.kPivotI);
      pivotPIDController.setD(IntakeConstants.kPivotD);
      this.setGoal(IntakeConstants.kPivotAngleSpeaker);
    });
  }

}

// public PivotSubsystem() {

// }

// public Command intake() {
// return new InstantCommand(() -> {
// // 1. Move the pivot to the intake position
// pivotPIDController.setReference(IntakeConstants.kPivotAngleIntake,
// ControlType.kPosition);

// });
// }
// }
