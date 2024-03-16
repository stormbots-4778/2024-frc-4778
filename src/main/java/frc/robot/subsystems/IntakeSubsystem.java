package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.subsystems.PivotSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimelightConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final LauncherSubsystem launcherSubsystem;
    private final PivotSubsystem pivotSubsystem;
    

    public final CANSparkMax topRoller;
    private final SparkPIDController topRollerPIDController;

    public final CANSparkMax bottomRoller;
    private final SparkPIDController bottomRollerPIDController;


    public IntakeSubsystem(LauncherSubsystem launcherSubsystem, PivotSubsystem pivotSubsystem) {
        this.launcherSubsystem = launcherSubsystem;
        this.pivotSubsystem = pivotSubsystem;

        topRoller = new CANSparkMax(IntakeConstants.kTopRollerCanId, MotorType.kBrushless);
        bottomRoller = new CANSparkMax(IntakeConstants.kBottomRollerCanId, MotorType.kBrushless);
        

        // Reset motors to known starting point            
        topRoller.restoreFactoryDefaults();
        bottomRoller.restoreFactoryDefaults();
        

        topRoller.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        topRoller.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);

        topRollerPIDController = topRoller.getPIDController();
        topRollerPIDController.setP(IntakeConstants.intakeKp);
        
        bottomRoller.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        bottomRoller.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
        
        bottomRollerPIDController = bottomRoller.getPIDController();
        bottomRollerPIDController.setP(IntakeConstants.intakeKp);

        // // pivotMotor.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        // pivotMotor.setSmartCurrentLimit(IntakeConstants.kPivotMotorCurrentLimit);

        // // TODO: Confirm if we should be using the RelativeEncoder (above) or AbsoluteEncoder for this motor
        // //pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
       
        // pivotEncoder = pivotMotor.getEncoder();
      
        // pivotEncoder.setPositionConversionFactor(IntakeConstants.kPivotEncoderPositionFactor);
        // pivotEncoder.setVelocityConversionFactor(IntakeConstants.kPivotEncoderVelocityFactor);

        // pivotPIDController = pivotMotor.getPIDController();
        // pivotPIDController.setFeedbackDevice(pivotEncoder);
        // pivotPIDController.setPositionPIDWrappingEnabled(false);
        // pivotPIDController.setPositionPIDWrappingMinInput(IntakeConstants.kPivotEncoderPositionPIDMinInput);
        // pivotPIDController.setPositionPIDWrappingMaxInput(IntakeConstants.kPivotEncoderPositionPIDMaxInput);
        // pivotPIDController.setP(IntakeConstants.kPivotP);
        // pivotPIDController.setI(IntakeConstants.kPivotI);
        // pivotPIDController.setD(IntakeConstants.kPivotD);
        // pivotPIDController.setFF(IntakeConstants.kPivotFF);
        // // pivotPIDController.setClosed

        // pivotPIDController.setOutputRange(IntakeConstants.kPivotMinOutput, IntakeConstants.kPivotMaxOutput);


    
        // Save motor configuration
        topRoller.burnFlash();
        bottomRoller.burnFlash();

    }
    
    // State machine
    public Command intake() {
        return new InstantCommand(() -> {
            // 1. Move the pivot to the intake position
            // pivotPIDController.setReference(IntakeConstants.kPivotAngleIntake, ControlType.kPosition);
            //this is now moved to the pivot subsystem
            
            // 2. Run the intake motors until a note is loaded
            topRoller.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit / 4);
            bottomRoller.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit / 4);
            topRoller.set(IntakeConstants.intakeSpeed);
             
            bottomRoller.set(IntakeConstants.intakeSpeed);


            pivotSubsystem.setPivotGoalCommand(IntakeConstants.kPivotAngleIntake);
;
            // 3. Stop the launcher if it is running
            launcherSubsystem.stop();
        }, this, launcherSubsystem);
    }
    
    public Command outtake() {
        return runOnce(() -> {
            // 1. Run the intake motors in reverse
            topRoller.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
            bottomRoller.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
            topRoller.set(IntakeConstants.outtakeSpeed);

            bottomRoller.set(IntakeConstants.outtakeSpeed);

            // If we are in the speaker position, the launcher should already be running
        });
    }

       public Command ampShoot() {
        return runOnce(() -> {
            // 1. Run the intake motors in reverse
            topRoller.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
            bottomRoller.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
            topRoller.set(IntakeConstants.shootSpeedTop);
            bottomRoller.set(IntakeConstants.shootSpeedBottom);
            

            // If we are in the speaker position, the launcher should already be running
        });
    }


    public void autoAmpShoot() {
        topRoller.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
        bottomRoller.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
        topRoller.set(IntakeConstants.shootSpeedTop);
        bottomRoller.set(IntakeConstants.shootSpeedBottom);
    }

    public Command ampPosition() {
        return new InstantCommand(() -> {
            // 1. Move the pivot to the amp position
            
            //this is now moved to the pivot subsystem
            // 2. Stop the launcher if it is running
            pivotSubsystem.setPivotGoalCommand(IntakeConstants.kPivotAngleAmp);
            launcherSubsystem.stop();
        }, this, launcherSubsystem);
    }

    public Command speakerPosition() {
        return new InstantCommand(() -> {  // Should this be run(..) or runOnce(..)? Try and see
                // 1. Move the pivot to the speaker position
                
                pivotSubsystem.setPivotGoalCommand(IntakeConstants.kPivotAngleSpeaker);
                // 2. Start the launcher
                // launcherSubsystem.shoot();
            }, this, launcherSubsystem);
    }

    public Command stopIntake() {
        return runOnce(() -> {
            topRoller.set(0.0);
            bottomRoller.set(0.0);
        });
    }

    

    @Override
    public void periodic() {
        
        
       
    }
}