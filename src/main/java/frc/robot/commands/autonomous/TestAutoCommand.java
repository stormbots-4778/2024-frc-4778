package frc.robot.commands.autonomous;

import javax.sound.sampled.LineEvent;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.AutoAim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class TestAutoCommand extends Command {
    private final DriveSubsystem m_robotDrive;
    private final PivotSubsystem m_pivot;
    private final IntakeSubsystem m_intake;
    private final AutoAim m_autoaim;

    NetworkTable table;
    NetworkTableEntry pipeline;

    double tv;
    double tx;
    double ty;

    public double rangeOffset = 10;

    public static double rollerSpeedAdjust = 0;

    double ySpeed;
    double xSpeed;
    double rotSpeed;

    public TestAutoCommand(DriveSubsystem m_robotDrive, PivotSubsystem m_pivot, IntakeSubsystem m_intake, AutoAim m_autoaim) {
        this.m_robotDrive = m_robotDrive;
        this.m_pivot = m_pivot;
        this.m_intake = m_intake;
        this.m_autoaim = m_autoaim;

        addRequirements(m_robotDrive, m_pivot, m_intake, m_autoaim);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry pipeline = table.getEntry("pipeline");
        table.getEntry("ledMode").setNumber(3);
        pipeline.setNumber(0);

        // check against 'tv' before aligning
        tv = table.getEntry("tv").getDouble(0);
        tx = table.getEntry("tx").getDouble(0);
        ty = table.getEntry("ty").getDouble(0);

        double[] poseArray = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

        double angleError = 0.0;
        if (poseArray.length == 0) {
            System.out.println("AmpAlign: Empty limelight pose array");
        } else {
            angleError = poseArray[5];
        }

        ySpeed = 0.0;
        xSpeed = 0.0;
        rotSpeed = 0.0;

        m_pivot.setPivotGoalCommand(IntakeConstants.kPivotAngleAmp);

        // double yawCalculated = (Math.signum(Drive.m_gyro.getYaw()) * Math.PI) -
        // (Math.toRadians(Drive.m_gyro.getYaw()));

        // double kpTurn = 0.25;
        float KpStrafe = 0.015f;

        // if (Math.abs(Math.toDegrees(yawCalculated)) > 1 || Math.abs(xError) > .25){
        // rot = MathUtil.clamp((kpTurn * yawCalculated) + .05, -0.25, 0.25);

        rotSpeed = (KpStrafe * (angleError));
        ySpeed = -(KpStrafe * (tx)); // tx = horizontal error, strafe direction in robot coordinates
        xSpeed = (KpStrafe * (rangeOffset - ty));
        // }

        if (rotSpeed > 0.10) {
            rotSpeed = 0.10;
        } else if (rotSpeed < -0.10) {
            rotSpeed = -0.10;
        }

        if (xSpeed > 0.075) {
            xSpeed = 0.075;
        } else if (xSpeed < -0.075) {
            xSpeed = -0.075;
        }
        if (ySpeed > 0.1) {
            ySpeed = 0.1;
        } else if (ySpeed < -0.1) {
            ySpeed = -0.1;
        }

        if (tv < 0.9999) {
            xSpeed = 0;
            ySpeed = 0;
            rotSpeed = 0;
        }

        m_robotDrive.drive(xSpeed, ySpeed, rotSpeed, false, false);
    }

    @Override
    public void end(boolean interrupted) {
        table.getEntry("ledMode").setNumber(1);
    }

    @Override
    public boolean isFinished() {
        if((tv > 0.9999) && (Math.abs(rotSpeed) < 0.025) && (Math.abs(xSpeed) < 0.025)
                && (Math.abs(ySpeed) < 0.025)) {
            m_intake.autoAmpShoot();

            return true;
        }

        return false;
    }
}