package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherPivotSubsystem extends SubsystemBase {
    private final CANSparkMax pivotMotor;

    public LauncherPivotSubsystem() {
        pivotMotor = new CANSparkMax(26, MotorType.kBrushless);
    }

    public void pivotUp() {
        pivotMotor.set(0.5);
    }

    public void pivotDown() {
        pivotMotor.set(-0.5);
    }

    public void stopPivot() {
        pivotMotor.set(0.0);
    }
}