// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.LauncherPivotSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private BlinkinSubsystem m_blinkin;



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    


    m_robotContainer = new RobotContainer();




    CameraServer.startAutomaticCapture();

    

    



    

    /* Configure Pigeon2 */

    // var toApply = new Pigeon2Configuration();

    // pidgey.getConfigurator().apply(toApply);

    // /* Speed up signals to an appropriate rate */
    // pidgey.getYaw().setUpdateFrequency(100);
    // pidgey.getGravityVectorZ().setUpdateFrequency(100);
  

  /* User can change the configs if they want, or leave it empty for factory-default */

  

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Launcher Pivot Position", LauncherPivotSubsystem.launcherPivotEncoder.getPosition());
    SmartDashboard.putNumber("Intake Pivot Position", PivotSubsystem.pivotEncoder.getPosition());
    
    // if (Timer.getFPGATimestamp() - currentTime > 1) {
    //   currentTime += 1;

      /**
       * getYaw automatically calls refresh(), no need to manually refresh.
       * 
       * StatusSignalValues also have the toString method implemented, to provide
       * a useful print of the signal.
       */
      // var yaw = pidgey.getYaw();
      // System.out.println("Yaw is " + yaw.toString() + " with " + yaw.getTimestamp().getLatency() + " seconds of latency");

      /**
       * Get the gravity vector Z component StatusSignalValue
       */
      // var gravityVectorZ = pidgey.getGravityVectorZ();
      /* This time wait for the signal to reduce latency */
      // gravityVectorZ.waitForUpdate(1); // Wait up to our period
      /**
       * This uses the explicit getValue and getUnits functions to print, even though it's not
       * necessary for the ostream print
      //  */
      // System.out.println("Gravity Vector in the Z direction is " +
      //                    gravityVectorZ.getValue() + " " +
      //                    gravityVectorZ.getUnits() + " with " +
      //                    gravityVectorZ.getTimestamp().getLatency() + " seconds of latency");
      /**
       * Notice when running this example that the second print's latency is always shorter than the first print's latency.
       * This is because we explicitly wait for the signal using the waitForUpdate() method instead of using the refresh()
       * method, which only gets the last cached value (similar to how Phoenix v5 works).
       * This can be used to make sure we synchronously update our control loop from the CAN bus, reducing any latency or jitter in
       * CAN bus measurements.
       * When the device is on a CANivore, the reported latency is very close to the true latency of the sensor, as the CANivore
       * timestamps when it receives the frame. This can be further used for latency compensation.
       */
      // System.out.println();
    }
    






  

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.m_blinkin.blinkin1.set(0.57);


  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.m_blinkin.blinkin1.set(0.57);

    
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();

    }

    m_robotContainer.m_limelight.ampServoPos();

    m_robotContainer.m_pivot.intakePos(IntakeConstants.kPivotAngleSpeaker);


    //TODO: Remember to switch from a command to a method to set the lift in the beginning
    // LiftSubsystem.getInstance().setLiftGoalCommand(0);
    // m_lift.setLiftGoalCommand(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    

    

    
    m_robotContainer.m_limelight.ampServoPos();


    m_robotContainer.m_pivot.intakePos(IntakeConstants.kPivotAngleSpeaker);
    

    /**
     * When we teleop init, set the position of the Pigeon2 and wait for the setter to take affect.
     */
    // pidgey.setYaw(144, 0.1); // Set our yaw to 144 degrees and wait up to 100 ms for the setter to take affect
    // pidgey.getYaw().waitForUpdate(0.1); // And wait up to 100 ms for the position to take affect
    // System.out.println("Set the position to 144 degrees, we are currently at " + pidgey.getYaw()); // Use java's implicit toString operator






  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

   SmartDashboard.putNumber("Climber", m_robotContainer.m_lift.LiftEncoder.getPosition());
 

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.

    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
