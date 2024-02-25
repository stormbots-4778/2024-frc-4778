import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;


public class LiftSubsystem extends SubsystemBase {
    private final CANSparkMax liftMotor;
    private final XboxController controller;

    public LiftSubsystem(XboxController controller) {
        this.controller = controller;
        liftMotor = new CANSparkMax(25, MotorType.kBrushless);

        liftMotor.restoreFactoryDefaults();
        liftMotor.setIdleMode(liftConstants.kLiftMotorIdleMode);
        liftMotor.setSmartCurrentLimit(LiftConstants.kLiftMotorCurrentLimit);

        liftPIDController = leftShooter.getPIDController();
        liftPIDController.setP(LiftConstants.liftKp);

        liftMotor.burnFlash();
    }

  
        

    @Override
    public void periodic() {
        int dpadAngle = controller.getPOV();
        if (dpadAngle == 0) { // dpad up
            liftMotor.set(1.0);
        } else if (dpadAngle == 180) { // dpad down
            liftMotor.set(-1.0);
        } else {
            liftMotor.set(0.0);
        }
    }
}
