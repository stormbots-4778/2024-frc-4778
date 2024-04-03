package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class BlinkinSubsystem extends SubsystemBase{
    
  Spark blinkin1;
  Spark blinkin2;

  public BlinkinSubsystem() {
		blinkin1 = new Spark(1);
    blinkin2 = new Spark(2);
	}

public Command Confetti() {
  return run(() -> {
    blinkin1.set(-0.87);
  });
  
}

  // when robot is on, (wait for power to blinkin?) send blinkin 5v command



}
