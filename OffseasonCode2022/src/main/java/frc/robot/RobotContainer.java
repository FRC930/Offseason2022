// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  private final Joystick driver = new Joystick(0);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton extendIntake = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton intakeRollers = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

  /* Modules */
  public static final SwerveModuleConstants frontLeftModule = new SwerveModuleConstants(1, 5, 9, 188.251);
  public static final SwerveModuleConstants frontRightModule = new SwerveModuleConstants(3, 7, 11, 296.191);
  public static final SwerveModuleConstants backLeftModule = new SwerveModuleConstants(2, 6, 10, 320.705);
  public static final SwerveModuleConstants backRightModule = new SwerveModuleConstants(4, 8, 12, 135.077);

  /* Subsystems */
  private final Swerve m_Swerve = new Swerve(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  private final IntakeSubsystem m_IntakeSubsystem;

  /* Commands */
  private final ExtendIntakeCommand m_ExtendIntakeCommand;
  private final RunIntakeRollersCommand m_RunIntakeRollersCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // INTAKE INITS //
    // These are random ID numbers for testing
    m_IntakeSubsystem = new IntakeSubsystem(1, 2, 3);
    m_RunIntakeRollersCommand = new RunIntakeRollersCommand(m_IntakeSubsystem);
    m_ExtendIntakeCommand = new ExtendIntakeCommand(m_IntakeSubsystem);
    boolean fieldRelative = true;
    boolean openLoop = true;
    m_Swerve.setDefaultCommand(new TeleopSwerve(m_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

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
    zeroGyro.whenPressed(new InstantCommand(() -> m_Swerve.zeroGyro()));
    extendIntake.whileActiveOnce(m_ExtendIntakeCommand);
    intakeRollers.whileActiveOnce(m_RunIntakeRollersCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(m_Swerve);
  }
}
