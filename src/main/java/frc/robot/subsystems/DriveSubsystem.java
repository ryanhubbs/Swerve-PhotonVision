// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  //AHRS ahrs;
  AHRS ahrs = new AHRS(SPI.Port.kMXP); 
  private AHRS m_gyro = ahrs;
  
  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private SwerveDrivePoseEstimator m_poseEstimator;
  // private Limelight m_frontCamera = new Limelight(Constants.Vision.kFrontCameraName, Constants.Vision.kRobotToCam);
  // private Pose2d lastPose = null;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    //Rotation2d.fromDegrees(m_gyro.getFusedHeading()),
    m_gyro.getRotation2d(),
    getModulePositions()
  );

  

  // Create Field2d Variable
  private final Field2d m_field = new Field2d();
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
      // Pathplanner AutoBuilder Config
    AutoBuilder.configureHolonomic(
      this::getPose, // Pose2d supplier
      this::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      this::getChassisSpeeds,
      this::setChassisSpeeds,
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
        new PIDConstants(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD), // Translation PID constants
        new PIDConstants(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD), // Rotation PID constants
        DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
        (DriveConstants.kWheelBase / 39.37 ) / 2, // Drive base radius in meters. Distance from robot center to furthest module
        new ReplanningConfig() // Default path replanning config. See the API for the options
      // here
      ),
      this // The drive subsystem. Used to properly set the requirements of path following
    );

    m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      getModulePositions(),
      m_odometry.getPoseMeters()
    );

    // Initilize Field2d
    SmartDashboard.putData("Field", m_field);
  }

  // depriciated

  // public Command followPathCommand(String pathName){
  //   PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
  //   // You must wrap the path following command in a FollowPathWithEvents command in order for event markers to work
  //   return new FollowPathWithEvents(
  //     new FollowPathHolonomic(
  //       path,
  //       this::getPose, // Robot pose supplier
  //       this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
  //       this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
  //       new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
  //         new PIDConstants(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD), // Translation PID constants
  //         new PIDConstants(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD), // Rotation PID constants
  //         DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
  //         (DriveConstants.kWheelBase / 39.37 ) / 2, // Drive base radius in meters. Distance from robot center to furthest module
  //         new ReplanningConfig() // Default path replanning config. See the API for the options here
  //       ),
  //       this // Reference to this subsystem to set requirements
  //     ),
  //     path, // FollowPathWithEvents also requires the path
  //     this::getPose // FollowPathWithEvents also requires the robot pose supplier
  //   );
  //  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    // var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(m_gyro.getRotation2d(), getModulePositions());

    // System.out.println("Periodic Start");
    // Optional<EstimatedRobotPose> estRobotPose = m_frontCamera.getEstimatedPose();
    // estRobotPose.ifPresent(
    //     est -> {
    //       EstimatedRobotPose estRobotPoseData = estRobotPose.get();

    //       Pose2d pose = estRobotPoseData.estimatedPose.toPose2d();
    //       double timestamp = estRobotPoseData.timestampSeconds;

    //       System.out.println("Target in View");

    //       if (pose != lastPose) {
    //         m_poseEstimator.addVisionMeasurement(pose, timestamp);
    //         lastPose = pose;

    //         System.out.println("Adding Vision Measurement: " + pose + "; " + timestamp);
    //     }
    //   }
    // );
    System.out.println("Periodic End");

    m_field.setRobotPose(m_odometry.getPoseMeters());
    //m_field.getObject("traj").setTrajectory(m_trajectory);
    SmartDashboard.updateValues();
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    m_poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }


  /**
   * Returns the current chassis speed of the robot.
   *
   * @return The chassis speed.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Changes the current chassis speed of the robot.
   *
   * @param speeds Sets the chassis speed.
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(swerveModuleStates);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    //return m_odometry.getPoseMeters();
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
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
   * @param limiterEnabled Whether to limit the driving speed or not
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit, boolean limiterEnabled) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;
    
    SmartDashboard.putBoolean("Speed Limiter", limiterEnabled);
    SmartDashboard.putNumber("Gyro Heading", m_gyro.getFusedHeading());

    if (limiterEnabled) {
      xSpeed = xSpeed / DriveConstants.kLimiterModifier;
      ySpeed = ySpeed / DriveConstants.kLimiterModifier;
    }

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

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
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
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
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
    // System.out.println(m_gyro.getRotation2d());
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    // m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(m_gyro.getAngle())));
    // m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(m_gyro.getAngle())));
    // m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(m_gyro.getAngle())));
    // m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(m_gyro.getAngle())));

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

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
