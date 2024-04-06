package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;

public class LimelightSubsystem extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public NetworkTableEntry tx = table.getEntry("tx");
    public NetworkTableEntry ty = table.getEntry("ty");
    public NetworkTableEntry ta = table.getEntry("ta");
    public NetworkTableEntry ledMode = table.getEntry("ledMode");
    public static Servo LimelightServo;
    public static boolean isAmpAlignPos = true;

    public static boolean isSpeakerAlignPos = false;

    public void periodic() {
        LimelightConstants.x = tx.getDouble(0.0);
        LimelightConstants.y = ty.getDouble(0.0);
        LimelightConstants.area = ta.getDouble(0.0);

        SmartDashboard.putNumber("LimelightX", LimelightConstants.x);
        SmartDashboard.putNumber("LimelightY", LimelightConstants.y);
        SmartDashboard.putNumber("LimelightArea", LimelightConstants.area);

        SmartDashboard.putNumber("Limelight Servo Position", LimelightServo.getPosition());
    }

    public LimelightSubsystem() {
        LimelightServo = new Servo(0); // <======= change this number
    }

    public Command AmpAlignServoPos() {
        return runOnce(() -> {
            // 1. Run the intake motors in reverse
            LimelightServo.set(1);
            isAmpAlignPos = true;
            isSpeakerAlignPos = false; // <======= value needs to be tuned
        });
    }

    public Command SpeakerAlignServoPos() {
        return runOnce(() -> {
            // 1. Run the intake motors in reverse
            LimelightServo.set(0); 
            isAmpAlignPos = false;
            isSpeakerAlignPos = true;// <======= value needs to be tuned
            // LimelightServo.setPosition(0);
        });
    }

    public void ampServoPos(){
        LimelightServo.set(1);
    }



}
