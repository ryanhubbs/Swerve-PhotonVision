package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private DriveSubsystem m_drivetrain;

  private Limelight m_frontCamera;
  private Pose2d lastPose;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    m_drivetrain = m_robotContainer.m_robotDrive;
    m_frontCamera = m_robotContainer.m_frontCamera;

    lastPose = m_drivetrain.getPose();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }



  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}



  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}



  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    //System.out.println("Periodic Start");
    Optional<EstimatedRobotPose> estRobotPose = m_frontCamera.getEstimatedPose();
    estRobotPose.ifPresent(
        est -> {
          EstimatedRobotPose estRobotPoseData = estRobotPose.get();

          Pose2d pose = estRobotPoseData.estimatedPose.toPose2d();
          double timestamp = estRobotPoseData.timestampSeconds;

          System.out.println("Target in View");

          if (pose != lastPose) {
            m_drivetrain.addVisionMeasurement(pose, timestamp);
            lastPose = pose;

            System.out.println("Adding Vision Measurement: " + pose + "; " + timestamp);
        }
      }
    );
  }



  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
