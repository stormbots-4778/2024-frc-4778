package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAim extends SubsystemBase {
    
    public LimelightSubsystem Limelight;
    public DriveSubsystem Drive;
    private NetworkTable table;
    private double angleError;

    public AutoAim (LimelightSubsystem Limelight, DriveSubsystem Drive){
        this.Limelight = Limelight;
        this.Drive = Drive;
    }

    public Command AmpAlign() {
        return run(() -> {        
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
            
            //check against 'tv' before aligning
            double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
            double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
            double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
            double[] poseArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); 
            double angleError = poseArray[5];
            double ySpeed = 0.0;
            double xSpeed = 0.0;
            double rotSpeed = 0.0;
            
            //double yawCalculated = (Math.signum(Drive.m_gyro.getYaw()) * Math.PI) - (Math.toRadians(Drive.m_gyro.getYaw()));
            
            // double kpTurn = 0.25;
            float KpStrafe = 0.02f;
            
            //if (Math.abs(Math.toDegrees(yawCalculated)) > 1 || Math.abs(xError) > .25){
            //rot = MathUtil.clamp((kpTurn * yawCalculated) + .05, -0.25, 0.25);

            rotSpeed = (KpStrafe * (angleError + 1.9));
            ySpeed = -(KpStrafe * (tx + 4.7)); //tx = horizontal error, strafe direction in robot coordinates 
            xSpeed = (KpStrafe * (12.5 - ty)); 
            //}
            
            if (rotSpeed > 0.25){
                rotSpeed = 0.25;
            }
            else if (rotSpeed < -0.25){
                rotSpeed = -0.25;
            }
            
             if (xSpeed > 0.1){
                xSpeed = 0.1;
            }
            else if (xSpeed < -0.1){
                xSpeed = -0.1;
            }
             if (ySpeed > 0.1){
                ySpeed = 0.1;
            }
            else if (ySpeed < -0.1){
                ySpeed = -0.1;
            }

            if (tv < 1.0){
                xSpeed = 0;
                ySpeed = 0;
                rotSpeed = 0;
            }
            
            Drive.drive(xSpeed, ySpeed, rotSpeed, false, false);

            //debug test
            System.out.printf("%f\n", tv);
            System.out.printf("%f\n", tx);
            System.out.printf("%f\n", ty);
        });
    }

        







    
}