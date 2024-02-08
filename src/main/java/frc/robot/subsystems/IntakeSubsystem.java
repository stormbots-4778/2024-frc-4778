package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    // private CANSparkMax TopRoller, BottomRoller, PivotMotor;
    private CANSparkMax TopRoller, BottomRoller;
    private XboxController controller;

    public IntakeSubsystem(XboxController controller) {
        this.controller = controller;
        TopRoller = new CANSparkMax(20, MotorType.kBrushless);
        BottomRoller = new CANSparkMax(21, MotorType.kBrushless);
        // PivotMotor = new CANSparkMax(22, MotorType.kBrushless);
    }

    public void in() {
//        if (controller.getAButton()) {
            TopRoller.set(0.5);
            BottomRoller.set(0.5);
            // PivotMotor.set(0.5);
  //      }
    }

    public void out() {
 //       if (controller.getBButton()) {
            TopRoller.set(-0.5);
            BottomRoller.set(-0.5);
            // PivotMotor.set(-0.5);
        
        }

    public void deploy() {

    }

    public void stow() {

    }

    public void score() {

    }

}