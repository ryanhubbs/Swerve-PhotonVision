package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;



public class RobotContainer {
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  
  private SendableChooser<Command> m_autoChooser; 
  private Boolean limiterEnabled = false;

  public Limelight m_frontCamera = new Limelight(Constants.Vision.kFrontCameraName, Constants.Vision.kRobotToCam);


  public RobotContainer() {
    configureButtonBindings();

    // Register Pathplanner Commands
    NamedCommands.registerCommand("swerveBrake", Commands.run(() -> m_robotDrive.setX()));

    // Setup AutoBuilder SendableChooser
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);


    

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true, limiterEnabled),
            m_robotDrive));
  }


  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
          () -> 
              limiterEnabled = true
        ))
        .whileFalse(new RunCommand(
          () -> 
              limiterEnabled = false
        ));

    new JoystickButton(m_driverController, Button.kStart.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    // new JoystickButton(m_driverController, Button.kA.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.followPathCommand("Human Player").schedule(),
    //         m_robotDrive));

    // new JoystickButton(m_driverController, Button.kB.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.followPathCommand("Charge Right").schedule(),
    //         m_robotDrive));
  }

  


  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
