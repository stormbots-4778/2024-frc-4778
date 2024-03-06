package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class SpinLauncher extends SequentialCommandGroup{
    
    public SpinLauncher(LauncherSubsystem m_LauncherSubsystem) {
        addCommands(
           // pulsing puts the intake here anyways
           m_LauncherSubsystem.shoot()
        //    m_intake.speakerPosition(),
           
            
            



            );

            addRequirements(m_LauncherSubsystem);
        
    }
}