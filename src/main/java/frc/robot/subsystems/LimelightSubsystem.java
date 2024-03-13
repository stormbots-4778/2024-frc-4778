package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightSubsystem extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public NetworkTableEntry tx = table.getEntry("tx");
    public NetworkTableEntry ty = table.getEntry("ty");
    public NetworkTableEntry ta = table.getEntry("ta");

    public void periodic() {
        LimelightConstants.x = tx.getDouble(0.0);
        LimelightConstants.y = ty.getDouble(0.0);
        LimelightConstants.area = ta.getDouble(0.0);        

        SmartDashboard.putNumber("LimelightX", LimelightConstants.x);
        SmartDashboard.putNumber("LimelightY", LimelightConstants.y);
        SmartDashboard.putNumber("LimelightArea", LimelightConstants.area);
    }

    
}
