package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax TopRoller, BottomRoller, PivotMotor;
    private final SparkMaxPIDController TopRollerPIDController;
    private final SparkMaxPIDController BottomRollerPIDController;

    public IntakeSubsystem() {
        TopRoller = new CANSparkMax(IntakeConstants.kTopRollerCanId, MotorType.kBrushless);
        BottomRoller = new CANSparkMax(IntakeConstants.kBottomRollerCanId, MotorType.kBrushless);
        PivotMotor = new CANSparkMax(IntakeConstants.kIntakePivotCanId, MotorType.kBrushless);

            
            TopRoller.restoreFactoryDefaults();
            TopRoller.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
            TopRoller.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
    
            TopRollerPIDController = TopRoller.getPIDController();
            TopRollerPIDController.setP(IntakeConstants.intakeKp);
    
            TopRoller.burnFlash();
            
            
            BottomRoller.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
            BottomRoller.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
            
            BottomRollerPIDController = BottomRoller.getPIDController();
            BottomRollerPIDController.setP(ShooterConstants.shooterKp);
            
            BottomRoller.burnFlash();
        }
    

    // Roller commands
    public Command in() {
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

    public Command out() {
        return new InstantCommand(
                () -> {
                    TopRoller.set(IntakeConstants.outtakeSpeed);
                    BottomRoller.set(IntakeConstants.outtakeSpeed);
                },
                this).withTimeout(IntakeConstants.outtakeTime);
    }

    public Command stop() {
        return new InstantCommand(
                () -> {
                    TopRoller.set(0.0);
                    BottomRoller.set(0.0);
                },
                this);
    }

    // Arm commands

    public void deployPos() {
        PivotMotor.set(IntakeConstants.deployPivot);
    }

    public void stowPos() {
        PivotMotor.set(IntakeConstants.stowPivot);
    }

    public void ampPos() {
        PivotMotor.set(IntakeConstants.ampPivot);
    }

}