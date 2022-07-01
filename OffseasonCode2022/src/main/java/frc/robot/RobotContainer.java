// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AutoCommandManager.subNames;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.lib.util.SwerveModuleConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* Controllers */
    // Driver Controller
    private final ControllerManager m_driverController = new ControllerManager(0);
    // Codriver Controller
    private final ControllerManager m_codriverController = new ControllerManager(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Modules */
  public static final SwerveModuleConstants frontLeftModule = new SwerveModuleConstants(8, 9, 9, 188.251);
  public static final SwerveModuleConstants frontRightModule = new SwerveModuleConstants(11, 10, 11, 296.191);
  public static final SwerveModuleConstants backLeftModule = new SwerveModuleConstants(1, 0, 10, 320.705);
  public static final SwerveModuleConstants backRightModule = new SwerveModuleConstants(18, 19, 12, 135.077);

  /* Auto Command Manager */
  private final AutoCommandManager m_autoManager;

  /* Subsystems */
  private final Swerve m_Swerve = new Swerve(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  private final IntakeSubsystem m_IntakeSubsystem;

  /* Commands */
  private final ExtendIntakeCommand m_ExtendIntakeCommand;
  private final RunIntakeRollersCommand m_RunIntakeRollersCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Auto Command Manager Stuff
    m_autoManager = new AutoCommandManager();
    m_autoManager.addSubsystem(subNames.Swerve, m_Swerve);
    m_autoManager.initCommands();

    // INTAKE INITS //
    m_IntakeSubsystem = new IntakeSubsystem(2, 17, 3);
    m_RunIntakeRollersCommand = new RunIntakeRollersCommand(m_IntakeSubsystem);
    m_ExtendIntakeCommand = new ExtendIntakeCommand(m_IntakeSubsystem);
    boolean fieldRelative = true;
    boolean openLoop = true;
    m_Swerve.setDefaultCommand(new TeleopSwerve(m_Swerve, m_driverController.getController(), translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /* Driver Buttons */

    m_driverController.getYButton().whileActiveOnce(
        new InstantCommand(() -> m_Swerve.zeroGyro())
    );

    //Engages motors for intake when left bumper is pressed
    m_driverController.getLeftBumper().whileActiveOnce(
        m_ExtendIntakeCommand
    );

    //Runs the intake motors when right bumper is pressed
    m_driverController.getRightBumper().whileActiveOnce(
      m_RunIntakeRollersCommand  
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoManager.getAutonomousCommand();
  }
}
