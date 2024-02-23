package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    // private CANSparkMax TopRoller, BottomRoller, PivotMotor;
    private CANSparkMax TopRoller, BottomRoller, PivotMotor;
   // private XboxController controller;

    public IntakeSubsystem() {
 //       this.controller = controller;
        TopRoller = new CANSparkMax(Constants.kTopRollerCanId, MotorType.kBrushless);
        BottomRoller = new CANSparkMax(Constants.kBottomRollerCanId, MotorType.kBrushless);
         PivotMotor = new CANSparkMax(Constants.kIntakePivotCanId, MotorType.kBrushless);
    }


    // Roller commands
    // public Command in() {
    //     return new InstantCommand(
    //         ()-> {
    //             //TopRoller.set(0.25);
    //             //BottomRoller.set(0.25);
    //             TopRoller.setVoltage(0.1);
    //             BottomRoller.setVoltage(-0.1);
    //         }, 
    //         this).unless(() -> {
    //             // light sensor sees the game piece
    //             return false;
    //         });
    // }

    public Command in() {
        return new InstantCommand(
            ()-> {
                //TopRoller.set(0.25);
                //BottomRoller.set(0.25);
                TopRoller.setVoltage(0.1);
                BottomRoller.setVoltage(-0.1);
            });
            
    }

    public Command stop() {
        return new InstantCommand(
            ()-> {
                TopRoller.set(0);
                BottomRoller.set(0);
            });
    }

    public void out() {
            TopRoller.set(-0.25);
            BottomRoller.set(-0.25);      
        }

    public void shoot() {
            if (true){      // Add a check if the pivot is in the stowed or amp scoring position
            TopRoller.set(-0.5);
            BottomRoller.set(-0.5);        
            }
        }

    // public void stopRollers() {
    //         TopRoller.set(0);
    //         BottomRoller.set(0);
    //     }

    // Arm commands

    public void deployPos() {
        PivotMotor.set(0.5);
    }

    public void stowPos() {
        PivotMotor.set(-0.5);
    }

    public void ampPos() {
        PivotMotor.set(0.25);
    }

}