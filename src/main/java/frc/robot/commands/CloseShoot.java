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

public class CloseShoot extends SequentialCommandGroup{
    
    public CloseShoot(LauncherSubsystem m_LauncherSubsystem, IntakeSubsystem m_intake) {
        addCommands(
            new ParallelCommandGroup(
                //pulsing puts the intake here anyways
                new InstantCommand(() -> m_LauncherSubsystem.leftShooter.set(ShooterConstants.leftShootSpeed)),
                new InstantCommand(() -> m_LauncherSubsystem.rightShooter.set(ShooterConstants.rightShootSpeed))
            ),
              
                 
             
            
            new ParallelCommandGroup(
                new InstantCommand(() -> m_intake.topRoller.set(IntakeConstants.outtakeSpeed)),
                new InstantCommand(() -> m_intake.bottomRoller.set(IntakeConstants.outtakeSpeed))
            ).withTimeout(0.2) 
        
              
            



            );


        
    }
}


            