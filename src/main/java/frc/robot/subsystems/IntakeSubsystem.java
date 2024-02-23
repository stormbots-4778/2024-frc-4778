package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    // private CANSparkMax TopRoller, BottomRoller, PivotMotor;
    private CANSparkMax TopRoller, BottomRoller, PivotMotor;
   // private XboxController controller;

    public IntakeSubsystem() {
 //       this.controller = controller;
        TopRoller = new CANSparkMax(IntakeConstants.kTopRollerCanId, MotorType.kBrushless);
        BottomRoller = new CANSparkMax(IntakeConstants.kBottomRollerCanId, MotorType.kBrushless);
        PivotMotor = new CANSparkMax(IntakeConstants.kIntakePivotCanId, MotorType.kBrushless);
    }

    // Roller commands
    public Command in() {
        return new InstantCommand(
            ()-> {
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
                ()-> {
                    TopRoller.set(IntakeConstants.outtakeSpeed);
                    BottomRoller.set(IntakeConstants.outtakeSpeed);
                }, 
                this).withTimeout(IntakeConstants.outtakeTime);
        }

    public Command shoot() {
        return new InstantCommand(
            ()-> {
            TopRoller.set(IntakeConstants.shootSpeed);
            BottomRoller.set(IntakeConstants.shootSpeed);        
            },
            this).unless(() -> {
                // light sensor sees the game piece
                return false;} );
        }

    // public void stopRollers() {
    //         TopRoller.set(0);
    //         BottomRoller.set(0);
    //     }

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