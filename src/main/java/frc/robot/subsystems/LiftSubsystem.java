package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase {
    private CANSparkMax LiftMotor;
    private final SparkPIDController LiftPIDController;
   

    public LiftSubsystem() {
        LiftMotor = new CANSparkMax(LiftConstants.kLiftMotorCanId, MotorType.kBrushless);
        LiftMotor.restoreFactoryDefaults();
        //leftShooter.setIdleMode(ShooterConstants.kShootingMotorIdleMode);
        LiftMotor.setSmartCurrentLimit(LiftConstants.kLiftMotorCurrentLimit);

        LiftPIDController = LiftMotor.getPIDController();
        LiftPIDController.setP(LiftConstants.liftKp);

        LiftMotor.burnFlash();
        
        
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
