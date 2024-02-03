package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class DriveSubsystem extends SubsystemBase {

  public final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);

  public boolean autoBalanceToggle = false;
  public boolean wallAlignToggle = false;
  

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // m_gyro = new AHRS(SerialPort.Port.kUSB2);
    // m_gyro = new AHRS(SerialPort.Port.kUSB1);
    // m_gyro.enableLogging(true);
    // m_gyro.calibrate();
  }

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor


  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getYaw() * -1),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });




  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getYaw() * -1),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

      
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getYaw() * -1),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */


   double gyroYaw = m_gyro.getYaw();
   double gyroPitch = m_gyro.getPitch();
   double gyroRoll = -m_gyro.getRoll();

   double gyroPitchRad = Math.toRadians(gyroPitch);
   double gyroRollRad = Math.toRadians(gyroRoll);

   boolean toggle;
   boolean target = false;

   float KpStrafe = 0.0001f;
   float KpDistance = -0.1f;
   float min_aim_command = 0.05f;

   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

   //float tx = table.getEntry("tx");
   // float ty = table->GetNumber("ty");

   public double inputTranslationDir;
   public double inputTranslationMag;

   public boolean lastCentric;

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    if(!autoBalanceToggle && fieldRelative){
      lastCentric = true;
    } else if (!autoBalanceToggle && !fieldRelative){
      lastCentric = false;
    }

    if(autoBalanceToggle){

      // Default: gyroYaw = m_gyro.getYaw();
      gyroYaw = (-(Math.signum(m_gyro.getYaw()))) * (Math.signum(m_gyro.getYaw()) * 180) - m_gyro.getYaw();
      gyroPitch = m_gyro.getPitch();
      gyroRoll = -m_gyro.getRoll();

      gyroPitchRad = Math.toRadians(gyroPitch);
      gyroRollRad = -Math.toRadians(gyroRoll);

      if(gyroPitch <= -3 || gyroPitch >= 3 || gyroRoll <= -3 || gyroRoll >= 3){
        gyroPitchRad = Math.toRadians(gyroPitch);
        gyroRollRad = Math.toRadians(gyroRoll);

        inputTranslationDir = -Math.atan2(gyroRollRad, gyroPitchRad);
        if(Math.abs(gyroPitch) < 1.5){
          inputTranslationMag = (gyroPitchRad + gyroRollRad) / 3;
        } else {
          inputTranslationMag = (gyroPitchRad + gyroRollRad) / 2.75;
        }
        fieldRelative = false;
        rot = 0;
      } else {
        inputTranslationDir = 0;
        inputTranslationMag = 0;
      }
      
        rateLimit = true;
    } else if (LimelightConstants.limelightToggle){
      fieldRelative = false;

        double xError = LimelightConstants.x;
        double yError = -LimelightConstants.y;
        
        double yawCalculated = (Math.signum(m_gyro.getYaw()) * Math.PI) - (Math.toRadians(m_gyro.getYaw()));
        
        double kpTurn = 0.25;
        float KpStrafe = 0.01f;
        
        if (Math.abs(Math.toDegrees(yawCalculated)) > 1 || Math.abs(xError) > .25){
         //rot = MathUtil.clamp((kpTurn * yawCalculated) + .05, -0.25, 0.25);
         ySpeed = -(KpStrafe * xError);
        }

        // if (Math.abs(yError) >  0.25 ){
        //  xSpeed = (KpStrafe * yError);
        // }
      } else if (wallAlignToggle) {
        double KpTurn = 0.25;

        double yawTo0 = Math.toRadians(m_gyro.getYaw());
        double yawTo90 = (Math.signum(m_gyro.getYaw()) * (Math.PI / 2)) - (Math.toRadians(m_gyro.getYaw()));
        double yawTo180 = (Math.signum(m_gyro.getYaw()) * Math.PI) - (Math.toRadians(m_gyro.getYaw()));
        double targetSpeed = yawTo90 > yawTo180? yawTo180:yawTo90;
        targetSpeed = targetSpeed > yawTo0? yawTo0: targetSpeed;

        if(Math.abs(Math.toDegrees(targetSpeed)) < 1) {
          wallAlignToggle = false;
        } else {
          rot = MathUtil.clamp((targetSpeed * KpTurn) + .05, -0.25, 0.25);
        }
      }

    if(!autoBalanceToggle && lastCentric && !fieldRelative){
      lastCentric = false;
      fieldRelative = true;
    }


    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      if(!autoBalanceToggle){
        inputTranslationDir = Math.atan2(ySpeed, xSpeed);
        inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
      }


      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag) * Math.signum(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }


    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-m_gyro.getYaw()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_rearLeft.stop();
    m_rearRight.stop();
}

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getYaw();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void AutoBalance(){
    boolean drive = true;
    // autoBalanceToggle = false;
    gyroPitch = m_gyro.getPitch();
    while((gyroPitch >= -6 && gyroPitch <= 6)) {
      gyroPitch = m_gyro.getPitch();
      SmartDashboard.putNumber("PITCH AUTO", gyroPitch);
      drive(.1, 0, 0, false, true);
    }
    drive = false;
    autoBalanceToggle = true;

    while(drive) {
      gyroPitch = m_gyro.getPitch();
      drive(0, 0, 0, false, true);
    }
  }


  public void BalanceToggle(){
    autoBalanceToggle = !autoBalanceToggle;
  }

  public void WallAlign() {

  }


  String vardhan = "Stinky";
}
