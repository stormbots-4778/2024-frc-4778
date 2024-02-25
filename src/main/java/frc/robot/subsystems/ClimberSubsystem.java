package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax climberMotor;
    private final SparkPIDController climberPIDController;
   

    public ClimberSubsystem() {
        climberMotor = new CANSparkMax(ClimberConstants.kClimberCanId, MotorType.kBrushless);
        climberMotor.restoreFactoryDefaults();
        //leftShooter.setIdleMode(ShooterConstants.kShootingMotorIdleMode);
        climberMotor.setSmartCurrentLimit(ClimberConstants.kClimberMotorCurrentLimit);

        climberPIDController = climberMotor.getPIDController();
        climberPIDController.setP(ClimberConstants.climberKp);

        climberMotor.burnFlash();
        
        
    }

    // public Command shoot() {
    //     return runOnce(()-> {
    //         leftShooter.set(ShooterConstants.leftShootSpeed);
    //         rightShooter.set(ShooterConstants.rightShootSpeed);
    //     });
    // }

    // public Command () {
    //     return runOnce(()-> {
    //         leftShooter.set(0.0);
    //         rightShooter.set(0.0);        
    //     });
    // }
}
