package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;

public class IntakeSubsystem extends SubsystemBase {
    public enum IntakePosition {
        INTAKE(0.0),
        AMP(Math.PI/2.0),
        SPEAKER(Math.PI);

        public final double angle;

        private IntakePosition(double angle) {
            this.angle = angle;
        }
    }

    private CANSparkMax TopRoller;
    private final SparkMaxPIDController TopRollerPIDController;

    private CANSparkMax BottomRoller;
    private final SparkMaxPIDController BottomRollerPIDController;
    
    private CANSparkMax PivotMotor;
    private final SparkMaxPIDController pivotPIDController;
    private final AbsoluteEncoder pivotEncoder;

    public IntakeSubsystem() {
        TopRoller = new CANSparkMax(IntakeConstants.kTopRollerCanId, MotorType.kBrushless);
        BottomRoller = new CANSparkMax(IntakeConstants.kBottomRollerCanId, MotorType.kBrushless);
        PivotMotor = new CANSparkMax(IntakeConstants.kIntakePivotCanId, MotorType.kBrushless);

        // Reset motors to known starting point            
        TopRoller.restoreFactoryDefaults();
        BottomRoller.restoreFactoryDefaults();
        PivotMotor.restoreFactoryDefaults();


        TopRoller.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        TopRoller.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);

        TopRollerPIDController = TopRoller.getPIDController();
        TopRollerPIDController.setP(IntakeConstants.intakeKp);
        
        BottomRoller.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        BottomRoller.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
        
        BottomRollerPIDController = BottomRoller.getPIDController();
        BottomRollerPIDController.setP(IntakeConstants.intakeKp);

        PivotMotor.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        PivotMotor.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
        pivotPIDController = PivotMotor.getPIDController();
        pivotEncoder = PivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pivotPIDController.setFeedbackDevice(pivotEncoder);
        pivotEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        pivotEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
        pivotEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);
        pivotPIDController.setPositionPIDWrappingEnabled(true);
        pivotPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        pivotPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);
        pivotPIDController.setP(ModuleConstants.kTurningP);
        pivotPIDController.setI(ModuleConstants.kTurningI);
        pivotPIDController.setD(ModuleConstants.kTurningD);
        pivotPIDController.setFF(ModuleConstants.kTurningFF);
        pivotPIDController.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);
    
        // Save motor configuration
        TopRoller.burnFlash();
        BottomRoller.burnFlash();
        PivotMotor.burnFlash();
    }
    
    // State machine
    public Command intake() {
        return new InstantCommand(
            () -> {

            }, this);
    }
    
    public Command outtake() {
        return new InstantCommand(
            () -> {
                
            }, this);
    }

    public Command amp() {
        return new InstantCommand(
            () -> {
                
            }, this);
    }

    public Command speaker() {
        return new InstantCommand(
            () -> {
                
            }, this);
    }

    public IntakePosition currentPosition() {
        var pivotPosition = pivotEncoder.getPosition();
        
    }

    // Roller commands
    private Command in() {
        return new InstantCommand(
                () -> {
                    TopRoller.set(IntakeConstants.intakeSpeed);
                    BottomRoller.set(IntakeConstants.intakeSpeed);
                },
                this).unless(() -> {
                    // light sensor sees the game piece
                    return false;
                });
    }

    private Command out() {
        return new InstantCommand(
                () -> {
                    TopRoller.set(IntakeConstants.outtakeSpeed);
                    BottomRoller.set(IntakeConstants.outtakeSpeed);
                },
                this).withTimeout(IntakeConstants.outtakeTime);
    }

    private Command stop() {
        return new InstantCommand(
                () -> {
                    TopRoller.set(0.0);
                    BottomRoller.set(0.0);
                },
                this);
    }

    // Arm commands
    public void deployPos() {
        PivotMotor.set(IntakeConstants.deployPivot)
    }

    public void stowPos() {
        PivotMotor.set(IntakeConstants.stowPivot);
    }

    public void ampPos() {
        PivotMotor.set(IntakeConstants.ampPivot);
    }

}