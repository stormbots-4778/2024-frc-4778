package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.LauncherPivotSubsystem;
import frc.robot.subsystems.PivotSubsystem;
public class RobotPivotsSubsystem extends SubsystemBase{
    public LauncherPivotSubsystem LauncherPivot;
    public PivotSubsystem IntakePivot;



    public RobotPivotsSubsystem(LauncherPivotSubsystem LauncherPivot, PivotSubsystem IntakePivot){
        this.LauncherPivot = LauncherPivot;
        this.IntakePivot = IntakePivot;
    }


    public Command High() {
        return run(() -> {        
            

            
        LauncherPivot.setLauncherPivotGoalCommand(0); //<==================== value is wrong, don't know actual
        IntakePivot.setPivotGoalCommand(0); //<==================== value is wrong, don't know actual




        });

    }

    public Command Medium() {
        return run(() -> {        
            

            
        LauncherPivot.setLauncherPivotGoalCommand(0); //<==================== value is wrong, don't know actual
        IntakePivot.setPivotGoalCommand(0); //<==================== value is wrong, don't know actual




        });

    }


    public Command Low() {
        return run(() -> {        
            

            
        LauncherPivot.setLauncherPivotGoalCommand(0); //<==================== value is wrong, don't know actual
        IntakePivot.setPivotGoalCommand(0); //<==================== value is wrong, don't know actual




        });

    }






}
