package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LauncherPivotSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.RobotContainer;

public class RobotPivotsSubsystem extends SubsystemBase {
    public LauncherPivotSubsystem LauncherPivot;
    public PivotSubsystem IntakePivot;
    public boolean UpPosition;

    public RobotPivotsSubsystem(LauncherPivotSubsystem LauncherPivot, PivotSubsystem IntakePivot, boolean UpPosition) {
        this.LauncherPivot = LauncherPivot;
        this.IntakePivot = IntakePivot;
        this.UpPosition = UpPosition;
    }

    public Command High() {
        return run(() -> {
            LauncherPivot.setLauncherPivotGoalCommand(ShooterConstants.kLauncherPivotAngleHigh);
            IntakePivot.setPivotGoalCommand(IntakeConstants.kPivotAngleSpeaker);
        });

    }

    // public Command Move() {
    // return runOnce(() -> {

    // if (UpPosition){

    // } else {
    // //<======= Tune this value

    // }

    // UpPosition = !UpPosition;

    // });
    // }

    public Command Low() {
        return run(() -> {

            LauncherPivot.setLauncherPivotGoalCommand(ShooterConstants.kLauncherPivotAngleLow);
            IntakePivot.setPivotGoalCommand(IntakeConstants.kPivotAngleLow); // <======= Tune this value

        });

    }

    public Command trackLauncherPivot() {

        // final double command = 0.157895 * (LauncherPivotSubsystem.launcherPivotEncoder.getPosition()) + 10;

        // if (command < 0){
        //     command = 0;
        // }

        // if (command > IntakeConstants.kPivotAngleSpeaker){
        //     command = IntakeConstants.kPivotAngleSpeaker;
        // }

        return run(() -> { // Should this be run(..) or runOnce(..)? Try and see

            IntakePivot.setPivotGoalCommand(0.157895 * (LauncherPivotSubsystem.launcherPivotEncoder.getPosition()) + 10);

        });
    }

}
