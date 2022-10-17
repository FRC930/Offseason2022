// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.AutoCommandManager.subNames;
import frc.robot.commands.*;
import frc.robot.commands.shooterCommands.AdjustHoodCommand;
import frc.robot.commands.shooterCommands.ShooterCommand;
import frc.robot.subsystems.*;
import frc.robot.utilities.ShooterUtility;
import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.wpilibj.Compressor;
import frc.lib.util.IndexerSensorUtility;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

    // Compressor
    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

    /* Controllers */
    // Driver Controller
    private final ControllerManager m_driverController = new ControllerManager(0);
    // Codriver Controller
    private final ControllerManager m_codriverController = new ControllerManager(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Utilities */
    private final IndexerSensorUtility m_IndexerSensorUtility = new IndexerSensorUtility(16, 15, 3, 4);

    /* Auto Command Manager */
    private final AutoCommandManager m_autoManager = new AutoCommandManager() ;

    /* Modules */
    public static final SwerveModuleConstants frontLeftModule = new SwerveModuleConstants(8, 9, 9, 259.980);
    public static final SwerveModuleConstants frontRightModule = new SwerveModuleConstants(11, 10, 10, 233.877);
    public static final SwerveModuleConstants backLeftModule = new SwerveModuleConstants(1, 0, 0, 71.895);
    public static final SwerveModuleConstants backRightModule = new SwerveModuleConstants(18, 19, 19, 143.965);

    /* Subsystems */
    private final Swerve m_Swerve = new Swerve(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
    private final EndgamePistonSubsystem m_EndgamePistonSubsystem = new EndgamePistonSubsystem(0);
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem(2, 17, 3);
    private final ShooterMotorSubsystem m_ShooterMotorSubsystem = new ShooterMotorSubsystem(12, 7);
    private final ShooterHoodSubsystem m_ShooterHoodSubsystem = new ShooterHoodSubsystem(6);
    private final IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem(14, 13, 99);

    /** RobotContianer
     * The container for the robot. Contains subsystems, OI devices, and commands. 
    */
    public RobotContainer() {

        //  Compressor
        compressor.enableAnalog(100,115);

        //  Auto Command Manager Stuff
        m_autoManager.addSubsystem(subNames.Swerve, m_Swerve);
        m_autoManager.initCommands();

        // Drive INITS //
        boolean fieldRelative = true;       
        boolean openLoop = true;
        m_Swerve.setDefaultCommand(new TeleopSwerve(m_Swerve, m_driverController.getController(), translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

        // Configure the button bindings
        configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick
   * } or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //--------------------------------------------------------------------------
    /* Driver Buttons */
    m_driverController.getPOVUpTrigger().whileActiveOnce(
        new EngageEndgamePistonCommand(m_EndgamePistonSubsystem)
    );

    m_driverController.getPOVDownTrigger().whileActiveOnce(
        new EngageEndgamePistonCommand(m_EndgamePistonSubsystem)
    );
        
    m_driverController.getPOVDownTrigger().whileActiveOnce(
        new DisengageEndgamePistonCommand(m_EndgamePistonSubsystem)
    );

    m_driverController.getRightBumper().whileActiveOnce(
        new ShooterCommand(m_ShooterMotorSubsystem, m_IndexerSubsystem, 0.5)
    );

    m_driverController.getYButton().whileActiveOnce(
        new InstantCommand(() -> m_Swerve.zeroGyro())
    );

    //Engages motors for intake when left bumper is pressed
    m_driverController.getLeftBumper().whileActiveOnce(
        new ParallelCommandGroup(
            new ExtendIntakeCommand(m_IntakeSubsystem),
            new RunIntakeRollersCommand(m_IntakeSubsystem)
        )
    );

    //  shooter controls
    m_driverController.getRightBumper().whileActiveOnce(
        new ShooterCommand(m_ShooterMotorSubsystem, m_IndexerSubsystem, 0.5)
    );

    //--------------------------------------------------------------------------------
    /* Co-Driver Buttons */
    m_codriverController.getLeftBumper().whileActiveOnce(
        new ParallelCommandGroup(
            new IndexerCommand(m_IndexerSubsystem, m_IndexerSensorUtility, 0.5),
            new RunIntakeRollersCommand(m_IntakeSubsystem),
            new ExtendIntakeCommand(m_IntakeSubsystem)
        )
    );

    m_codriverController.getBButton().whileActiveOnce(
        new IndexerEjectCommand(m_IndexerSubsystem, 0.5)
    );

    // Tarmac
    m_codriverController.getPOVLeftTrigger().whileActiveOnce(
        new ParallelCommandGroup(
            new AdjustHoodCommand(
                m_ShooterHoodSubsystem,
                ShooterUtility.calculateHoodPos(9)
            ),
            new ShooterCommand(
                m_ShooterMotorSubsystem, 
                m_IndexerSubsystem,
                ShooterUtility.calculateTopSpeed(9),
                ShooterUtility.calculateBottomSpeed(9)
            )
        ).withTimeout(0.1)
    );

    // Launchpad
    m_codriverController.getPOVUpTrigger().whileActiveOnce(
        new ParallelCommandGroup(
            new AdjustHoodCommand(
                m_ShooterHoodSubsystem,
                ShooterUtility.calculateHoodPos(14.5)
            ),
            new ShooterCommand(
                m_ShooterMotorSubsystem, 
                m_IndexerSubsystem,
                ShooterUtility.calculateTopSpeed(14.5),
                ShooterUtility.calculateBottomSpeed(14.5)
            )
        ).withTimeout(0.1)
    );

    // Fender shot
    m_codriverController.getPOVDownTrigger().whileActiveOnce(
        new ParallelCommandGroup(
            new AdjustHoodCommand(
                m_ShooterHoodSubsystem,
                ShooterUtility.calculateHoodPos(19 / 12)
            ),
            new ShooterCommand(
                m_ShooterMotorSubsystem, 
                m_IndexerSubsystem,
                ShooterUtility.calculateTopSpeed(19 / 12),
                ShooterUtility.calculateBottomSpeed(19 / 12)
            )
        ).withTimeout(0.1)
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
