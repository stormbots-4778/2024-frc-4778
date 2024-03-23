package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ShooterConstants;

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
    private final CANSparkMax launcherPivotMotor = new CANSparkMax(ShooterConstants.kLauncherPivotCanId, MotorType.kBrushless);
    private final SparkPIDController launcherPivotPIDController;
    public static RelativeEncoder launcherPivotEncoder;

    public LauncherPivotSubsystem() {
        super(
            new TrapezoidProfile.Constraints(
                200.0, 200.0),0.0);
        
        launcherPivotMotor.restoreFactoryDefaults();
        launcherPivotMotor.getEncoder().setPosition(0.0);

        launcherPivotMotor.setIdleMode(ShooterConstants.kLauncherPivotMotorIdleMode);
        launcherPivotMotor.setSmartCurrentLimit(ShooterConstants.kLauncherPivotMotorCurrentLimit);

        launcherPivotEncoder = launcherPivotMotor.getEncoder();

        launcherPivotEncoder.setPositionConversionFactor(ShooterConstants.kLauncherPivotEncoderPositionFactor);
        launcherPivotEncoder.setVelocityConversionFactor(ShooterConstants.kLauncherPivotEncoderVelocityFactor);

        launcherPivotPIDController = launcherPivotMotor.getPIDController();
        launcherPivotPIDController.setFeedbackDevice(launcherPivotEncoder);
        launcherPivotPIDController.setPositionPIDWrappingEnabled(false);
        launcherPivotPIDController.setPositionPIDWrappingMinInput(ShooterConstants.kLauncherPivotEncoderPositionPIDMinInput);
        launcherPivotPIDController.setPositionPIDWrappingMaxInput(ShooterConstants.kLauncherPivotEncoderPositionPIDMaxInput);
        launcherPivotPIDController.setP(ShooterConstants.kLauncherPivotP);
        launcherPivotPIDController.setI(ShooterConstants.kLauncherPivotI);
        launcherPivotPIDController.setD(ShooterConstants.kLauncherPivotD);
        launcherPivotPIDController.setFF(ShooterConstants.kLauncherPivotFF);

        launcherPivotMotor.burnFlash();
    }

    @Override
    public void useState(TrapezoidProfile.State setpoint) {
        launcherPivotPIDController.setReference(setpoint.position, ControlType.kPosition);
    }

    public Command setLauncherPivotGoalCommand(double pivotPos) {
        return Commands.runOnce(() -> setGoal(pivotPos), this);
    }

    
}