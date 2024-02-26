package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final LauncherSubsystem launcherSubsystem;

    public final CANSparkMax topRoller;
    private final SparkPIDController topRollerPIDController;

    public final CANSparkMax bottomRoller;
    private final SparkPIDController bottomRollerPIDController;
    
    private final CANSparkMax pivotMotor;
    private final SparkPIDController pivotPIDController;
    public static RelativeEncoder pivotEncoder;

    public IntakeSubsystem(LauncherSubsystem launcherSubsystem) {
        this.launcherSubsystem = launcherSubsystem;

        topRoller = new CANSparkMax(IntakeConstants.kTopRollerCanId, MotorType.kBrushless);
        bottomRoller = new CANSparkMax(IntakeConstants.kBottomRollerCanId, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(IntakeConstants.kIntakePivotCanId, MotorType.kBrushless);

        // Reset motors to known starting point            
        topRoller.restoreFactoryDefaults();
        bottomRoller.restoreFactoryDefaults();
        pivotMotor.restoreFactoryDefaults();
        pivotMotor.getEncoder().setPosition(0.0);

        topRoller.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        topRoller.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);

        topRollerPIDController = topRoller.getPIDController();
        topRollerPIDController.setP(IntakeConstants.intakeKp);
        
        bottomRoller.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        bottomRoller.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
        
        bottomRollerPIDController = bottomRoller.getPIDController();
        bottomRollerPIDController.setP(IntakeConstants.intakeKp);

        // pivotMotor.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
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
        // pivotPIDController.setClosed

        pivotPIDController.setOutputRange(IntakeConstants.kPivotMinOutput, IntakeConstants.kPivotMaxOutput);
    
        // Save motor configuration
        topRoller.burnFlash();
        bottomRoller.burnFlash();
        pivotMotor.burnFlash();
    }
    
    // State machine
    public Command intake() {
        return new InstantCommand(() -> {
            // 1. Move the pivot to the intake position
            pivotPIDController.setReference(IntakeConstants.kPivotAngleIntake, ControlType.kPosition);

            // 2. Run the intake motors until a note is loaded
            topRoller.set(IntakeConstants.intakeSpeed);
            bottomRoller.set(IntakeConstants.intakeSpeed);

            // 3. Stop the launcher if it is running
            launcherSubsystem.stop();
        }, this, launcherSubsystem);
    }
    
    public Command outtake() {
        return runOnce(() -> {
            // 1. Run the intake motors in reverse
            topRoller.set(IntakeConstants.outtakeSpeed);
            bottomRoller.set(IntakeConstants.outtakeSpeed);

            // If we are in the speaker position, the launcher should already be running
        });
    }

    public Command ampPosition() {
        return new InstantCommand(() -> {
            // 1. Move the pivot to the amp position
            pivotPIDController.setReference(IntakeConstants.kPivotAngleAmp, ControlType.kPosition);

            // 2. Stop the launcher if it is running
            launcherSubsystem.stop();
        }, this, launcherSubsystem);
    }

    public Command speakerPosition() {
        return new InstantCommand(() -> {  // Should this be run(..) or runOnce(..)? Try and see
                // 1. Move the pivot to the speaker position
                pivotPIDController.setReference(IntakeConstants.kPivotAngleSpeaker, ControlType.kPosition);

                // 2. Start the launcher
                launcherSubsystem.shoot();
            }, this, launcherSubsystem);
    }

    public Command stopIntake() {
        return runOnce(() -> {
            topRoller.set(0.0);
            bottomRoller.set(0.0);
        });
    }

    public Command stopPivot() {
        return runOnce(() -> {
            pivotMotor.set(0.0);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Relative Encoder", pivotEncoder.getPosition());
    }
}