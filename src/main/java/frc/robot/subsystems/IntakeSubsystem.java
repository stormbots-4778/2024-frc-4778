package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    // private CANSparkMax TopRoller, BottomRoller, PivotMotor;
    private CANSparkMax TopRoller, BottomRoller, PivotMotor;
   // private XboxController controller;

    public IntakeSubsystem() {
 //       this.controller = controller;
        TopRoller = new CANSparkMax(20, MotorType.kBrushless);
        BottomRoller = new CANSparkMax(21, MotorType.kBrushless);
         PivotMotor = new CANSparkMax(22, MotorType.kBrushless);
    }


    // Roller commands
    public void in() {
            TopRoller.set(0.25);
            BottomRoller.set(0.25);
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

    public void stopRollers() {
            TopRoller.set(0);
            BottomRoller.set(0);
        }

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