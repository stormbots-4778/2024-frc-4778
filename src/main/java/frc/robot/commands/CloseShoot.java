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
           // pulsing puts the intake here anyways
           m_LauncherSubsystem.shoot(),
           m_intake.speakerPosition(),
           m_intake.outtake().withTimeout(1.0)
            



            );

            addRequirements(m_LauncherSubsystem, m_intake);
        
    }
}


            