package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LauncherSubsystem {
    public Command in() {
        return new InstantCommand(
            ()-> {
                //TopRoller.set(0.25);
                //BottomRoller.set(0.25);
                TopRoller.setVoltage(0.1);
                BottomRoller.setVoltage(-0.1);
            });
            
    }




    m_shooterPIDController.setP(ModuleConstants.kShootingP);
    m_drivingPIDController.setI(ModuleConstants.kDrivingI);
    m_drivingPIDController.setD(ModuleConstants.kDrivingD);
    m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
    m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);
}
