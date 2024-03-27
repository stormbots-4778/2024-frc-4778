package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
// import frc.robot.subsystems.LauncherPivotSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class AutoAim extends SubsystemBase {
    
    public LimelightSubsystem Limelight;
    public DriveSubsystem Drive;
    public IntakeSubsystem Intake;
    public PivotSubsystem Pivot;
    // public RobotPivotsSubsystem RobotPivot;
    public LauncherSubsystem Launcher;


    public AutoAim (LimelightSubsystem Limelight, DriveSubsystem Drive, IntakeSubsystem Intake, PivotSubsystem Pivot){
        this.Limelight = Limelight;
        this.Drive = Drive;
        this.Intake = Intake;
        this.Pivot = Pivot;
        // this.RobotPivot = RobotPivot;

    }

    public Command AmpAlign() {
        return run(() -> {        
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

            Limelight.AmpAlignServoPos(); //<===== turns limelight to amp side for alignment
            
            //check against 'tv' before aligning
            double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
            double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
            double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
            double[] poseArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); 
            double angleError = poseArray[5];
            double ySpeed = 0.0;
            double xSpeed = 0.0;
            double rotSpeed = 0.0;

            Pivot.setPivotGoalCommand(IntakeConstants.kPivotAngleAmp);
            
            //double yawCalculated = (Math.signum(Drive.m_gyro.getYaw()) * Math.PI) - (Math.toRadians(Drive.m_gyro.getYaw()));
            
            // double kpTurn = 0.25;
            float KpStrafe = 0.015f;
            
            //if (Math.abs(Math.toDegrees(yawCalculated)) > 1 || Math.abs(xError) > .25){
            //rot = MathUtil.clamp((kpTurn * yawCalculated) + .05, -0.25, 0.25);

            rotSpeed = (KpStrafe * (angleError + 2.5));
            ySpeed = -(KpStrafe * (tx + 4.7)); //tx = horizontal error, strafe direction in robot coordinates 
            xSpeed = (KpStrafe * (13 - ty)); 
            //}
            
            if (rotSpeed > 0.10){
                rotSpeed = 0.10;
            }
            else if (rotSpeed < -0.10){
                rotSpeed = -0.10;
            }
            
             if (xSpeed > 0.075){
                xSpeed = 0.075;
            }
            else if (xSpeed < -0.075){
                xSpeed = -0.075;
            }
             if (ySpeed > 0.1){
                ySpeed = 0.1;
            }
            else if (ySpeed < -0.1){
                ySpeed = -0.1;
            }

            if (tv < 0.9999){
                xSpeed = 0;
                ySpeed = 0;
                rotSpeed = 0;
            }
            
            Drive.drive(xSpeed, ySpeed, rotSpeed, false, false);
            
            if ( (tv > 0.9999) && (Math.abs(rotSpeed) < 0.025) && (Math.abs(xSpeed) < 0.025) && (Math.abs(ySpeed) < 0.025)){
                // Commands.runOnce(() -> {
                    
                    Intake.autoAmpShoot();

                // }, Intake);
            }

            //debug test
            System.out.printf("Rot, X, Y Speeds");
            System.out.printf("%f\n", rotSpeed);
            System.out.printf("%f\n", xSpeed);
            System.out.printf("%f\n", ySpeed);
        });
    }    
}