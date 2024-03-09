package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.LauncherConstants;
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

public class LauncherPivotSubsystem extends TrapezoidProfileSubsystem{
    private final CANSparkMax pivotMotor = new CANSparkMax(LauncherConstants.kLauncherPivotCanId, MotorType.kBrushless);
    private final SparkPIDController pivotPIDController;
    public static RelativeEncoder pivotEncoder;

    public LauncherPivotSubsystem() {
        super(
            new TrapezoidProfile.Constraints(
                200.0, 320.0),0.0);
        
        pivotMotor.restoreFactoryDefaults();
        pivotMotor.getEncoder().setPosition(0.0);

        pivotMotor.setIdleMode(LauncherConstants.kLauncherMotorIdleMode);
        pivotMotor.setSmartCurrentLimit(LauncherConstants.kPivotMotorCurrentLimit);

        pivotEncoder = pivotMotor.getEncoder();

        pivotEncoder.setPositionConversionFactor(LauncherConstants.kPivotEncoderPositionFactor);
        pivotEncoder.setVelocityConversionFactor(LauncherConstants.kPivotEncoderVelocityFactor);

        pivotPIDController = pivotMotor.getPIDController();
        pivotPIDController.setFeedbackDevice(pivotEncoder);
        pivotPIDController.setPositionPIDWrappingEnabled(false);
        pivotPIDController.setPositionPIDWrappingMinInput(LauncherConstants.kPivotEncoderPositionPIDMinInput);
        pivotPIDController.setPositionPIDWrappingMaxInput(LauncherConstants.kPivotEncoderPositionPIDMaxInput);
        pivotPIDController.setP(LauncherConstants.kPivotP);
        pivotPIDController.setI(LauncherConstants.kPivotI);
        pivotPIDController.setD(LauncherConstants.kPivotD);
        pivotPIDController.setFF(LauncherConstants.kPivotFF);

        pivotMotor.burnFlash();
    }

    @Override
    public void useState(TrapezoidProfile.State setpoint) {
        pivotPIDController.setReference(setpoint.position, ControlType.kPosition);
    }

    public Command setPivotGoalCommand(double pivotPos) {
        return Commands.runOnce(() -> setGoal(pivotPos), this);
    }
}