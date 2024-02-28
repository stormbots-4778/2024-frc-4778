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

public class PivotSubsystem extends TrapezoidProfileSubsystem{
    private final CANSparkMax pivotMotor = new CANSparkMax(IntakeConstants.kIntakePivotCanId, MotorType.kBrushless);
    private final SparkPIDController pivotPIDController;
    public static RelativeEncoder pivotEncoder;


    public PivotSubsystem() {

        super(
            new TrapezoidProfile.Constraints(
                120.0, 400.0),0.0);
        
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

//     public PivotSubsystem() {
        

     

      
        


//     }



//     public Command intake() {
//         return new InstantCommand(() -> {
//             // 1. Move the pivot to the intake position
//             pivotPIDController.setReference(IntakeConstants.kPivotAngleIntake, ControlType.kPosition);
           
//         });
//     }
// }
