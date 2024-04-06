package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.AutoShoot;
import frc.robot.subsystems.AutoAim;

public class BlinkinSubsystem extends SubsystemBase {

  public Spark blinkin1;
  public Spark blinkin2;

  public BlinkinSubsystem() {
    blinkin1 = new Spark(1);
    blinkin2 = new Spark(2);
  }

  public void Confetti() {

    blinkin1.set(0);

  }

  public void Default() {

    blinkin1.set(0.53);
    blinkin2.set(0.53);

  }

  public void Lock() {

    blinkin1.set(0.07);
    blinkin2.set(0.27);

  }

  public Command RedOrangeIntake() {
    return runOnce(() -> {
      blinkin1.set(0.63);
      blinkin2.set(0.63);
    });
  }

  public Command goldShoot() {
    return runOnce(() -> {
      blinkin1.set(0.67);
      blinkin2.set(0.67);
    });
  }

  public Command Rainbow() {
    return runOnce(() -> {
      blinkin1.set(0.51);
      blinkin2.set(0.51);
    });
  }

public Command runState(){
  return run(() -> {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
            NetworkTableEntry pipeline = table.getEntry("pipeline");
            
            // check against 'tv' before aligning
            double tv = table.getEntry("tv").getDouble(0);
            double tx = table.getEntry("tx").getDouble(0);
            double ty = table.getEntry("ty").getDouble(0);
    blinkin1.set(0.53);
    blinkin2.set(0.53);


    if (tv > 0.999){
      blinkin1.set(0.07);
      blinkin2.set(0.27);
    }


  });
}

  // when robot is on, (wait for power to blinkin?) send blinkin 5v command

}
