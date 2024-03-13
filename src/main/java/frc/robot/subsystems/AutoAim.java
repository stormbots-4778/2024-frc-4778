package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAim extends SubsystemBase {
    
    public LimelightSubsystem Limelight;
    public DriveSubsystem Drive;

    public AutoAim (LimelightSubsystem Limelight, DriveSubsystem Drive){
        this.Limelight = Limelight;
        this.Drive = Drive;
    }

    public Command AmpAlign() {
        return runOnce(() -> {        
            double xError = LimelightConstants.x;
            double yError = -LimelightConstants.y;
            double ySpeed = 0.0;
            double xSpeed = 0.0;
            
            double yawCalculated = (Math.signum(Drive.m_gyro.getYaw()) * Math.PI) - (Math.toRadians(Drive.m_gyro.getYaw()));
            
            // double kpTurn = 0.25;
            float KpStrafe = 0.01f;
            
            if (Math.abs(Math.toDegrees(yawCalculated)) > 1 || Math.abs(xError) > .25){
            //rot = MathUtil.clamp((kpTurn * yawCalculated) + .05, -0.25, 0.25);
            ySpeed = -(KpStrafe * xError);
            }

            if (Math.abs(yError) >  12.5){
            xSpeed = (KpStrafe * yError);
            }

            Drive.drive(xSpeed, ySpeed, 0.0, false, true);
        });
    }
}