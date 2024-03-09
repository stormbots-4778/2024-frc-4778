package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase {
    private CANSparkMax LiftMotor;
    private final SparkPIDController LiftPIDController;
    public static RelativeEncoder LiftEncoder;
   

    public LiftSubsystem() {
        LiftMotor = new CANSparkMax(LiftConstants.kLiftMotorCanId, MotorType.kBrushless);
        LiftMotor.restoreFactoryDefaults();
       
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

    public Command extend() {
        return runOnce(()-> {
            LiftMotor.set(LiftConstants.liftMotorSpeed);
        });
    }

    public Command retract() {
        return runOnce(()-> {
            LiftMotor.set(-(LiftConstants.liftMotorSpeed));
        });
    }

    public Command stop() {
        return runOnce(()-> {
            LiftMotor.set(0.0);
              
        });
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
