package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ShooterConstants;

public class LauncherSubsystem {

    public CANSparkMax leftShooter, rightShooter;

    // May not need because notes are squishy (would be speed controller)
    public PIDController shooterPID = new PIDController(ShooterConstants.shooterKp, ShooterConstants.shooterKi, ShooterConstants.shooterKd);

    public LauncherSubsystem() {
        leftShooter = new CANSparkMax(ShooterConstants.kleftShooterCanId, MotorType.kBrushless);
        rightShooter = new CANSparkMax(ShooterConstants.kleftShooterCanId, MotorType.kBrushless);
    }

    public void shoot() {
        leftShooter.set(ShooterConstants.shootSpeed);
        rightShooter.set(ShooterConstants.shootSpeed);
    }

    public void stop() {
        leftShooter.set(0);
        rightShooter.set(0);
    }
    
}
