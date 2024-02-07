package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax intakeMotor1, intakeMotor2, intakeMotor3;
    private XboxController controller;

    public IntakeSubsystem(XboxController controller) {
        this.controller = controller;
        intakeMotor1 = new CANSparkMax(1, MotorType.kBrushless);  // Caelen needs to replace these with actual port numbers
        intakeMotor2 = new CANSparkMax(2, MotorType.kBrushless);
        intakeMotor3 = new CANSparkMax(3, MotorType.kBrushless);
    }

    public void intake() {
        if (controller.getAButton()) {
            intakeMotor1.set(0.5);
            intakeMotor2.set(0.5);
            intakeMotor3.set(0.5);
        }
    }

    public void outtake() {
        if (controller.getBButton()) {
            intakeMotor1.set(-0.5);
            intakeMotor2.set(-0.5);
            intakeMotor3.set(-0.5);
        }
    }
}