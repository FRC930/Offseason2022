// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AutoCommandManager.subNames;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.shooterCommands.AdjustHoodCommand;
import frc.robot.commands.shooterCommands.ShooterCommand;
import frc.robot.subsystems.*;
import frc.robot.utilities.ShooterUtility;
import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.Compressor;
import frc.lib.util.IndexerSensorUtility;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

    /* Constants */
    double IndexerMotorSpeed = 0.25; //TODO 0.5;

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
    public static final SwerveModuleConstants frontLeftModule = new SwerveModuleConstants(8, 9, 9, 259.980);
    public static final SwerveModuleConstants frontRightModule = new SwerveModuleConstants(11, 10, 10, 233.877);
    public static final SwerveModuleConstants backLeftModule = new SwerveModuleConstants(1, 0, 0, 71.895);
    public static final SwerveModuleConstants backRightModule = new SwerveModuleConstants(18, 19, 19, 143.965);

    /* Auto Command Manager */
    private final AutoCommandManager m_autoManager;

    /* Subsystems */
    private final Swerve m_Swerve = new Swerve(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
    // private final IntakeSubsystem m_IntakeSubsystem;
    // private final ShooterMotorSubsystem m_ShooterMotorSubsystem;
    // private final ShooterHoodSubsystem m_ShooterHoodSubsystem;
    // private final IndexerSubsystem m_IndexerSubsystem;

    NeoMotorSubsystem m_stagedMotor;
    NeoMotorSubsystem m_ejectionMotor;
    NeoMotorSubsystem m_cargoCenteringMotor;
    NeoMotorSubsystem m_intakeRollerMotor;
    NeoMotorSubsystem m_loadedMotor;

    TalonMotorSubsystem m_shooterMotorLeft;
    TalonMotorSubsystem m_shooterMotorRight;
    

    /* Commands */
    // private final ExtendIntakeCommand m_ExtendIntakeCommand;
    // private final RunIntakeRollersCommand m_RunIntakeRollersCommand;
    // private final IndexerCommand m_IndexerCommand;
    // private final IndexerEjectCommand m_IndexerEjectCommand;
    // private final ShooterCommand m_ShooterCommand;
    // private final AdjustHoodCommand m_ShooterHoodCommand;

    NeoIndividualMotorCommand m_stagedMotorCommand;
    NeoIndividualMotorCommand m_stagedMotorReversedCommand;
    NeoIndividualMotorCommand m_ejectionMotorCommand;
    NeoIndividualMotorCommand m_ejectionMotorReversedCommand;
    NeoIndividualMotorCommand m_cargoCenteringMotorCommand;
    NeoIndividualMotorCommand m_cargoCenteringMotorReversedCommand;
    NeoIndividualMotorCommand m_intakeRollerMotorCommand;
    NeoIndividualMotorCommand m_intakeRollerMotorReversedCommand;
    NeoIndividualMotorCommand m_loadedMotorCommand;
    NeoIndividualMotorCommand m_loadedMotorReversedCommand;

    TalonIndividualMotorCommand m_shootMotorCommandLeft;
    TalonIndividualMotorCommand m_shootMotorCommandRight;

    /* Utilities */
    //private final IndexerSensorUtility m_IndexerSensorUtility;

    
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // PORT FORWARDING //
    PortForwarder.add(5800, "10.99.30.25", 5800);
    PortForwarder.add(1181, "10.99.30.25", 1181);
    PortForwarder.add(1182, "10.99.30.25", 1182);
    PortForwarder.add(1183, "10.99.30.25", 1183);
    PortForwarder.add(1184, "10.99.30.25", 1184);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    
    compressor.enableAnalog(100,115);
    
    

        // Auto Command Manager Stuff
        m_autoManager = new AutoCommandManager();
        m_autoManager.addSubsystem(subNames.Swerve, m_Swerve);
        m_autoManager.initCommands();

        // INDEXER INITS //
        // m_IndexerSubsystem = new IndexerSubsystem(14, 13, 99);
        // m_IndexerSensorUtility = new IndexerSensorUtility(3, 15, 16, 4);
        // m_IndexerCommand = new IndexerCommand(m_IndexerSubsystem, m_IndexerSensorUtility, IndexerMotorSpeed);
        // m_IndexerEjectCommand = new IndexerEjectCommand(m_IndexerSubsystem, IndexerMotorSpeed);

        // MOTORS //
        m_stagedMotor = new NeoMotorSubsystem(16);
        m_ejectionMotor = new NeoMotorSubsystem(4);
        m_cargoCenteringMotor = new NeoMotorSubsystem(3);
        m_intakeRollerMotor = new NeoMotorSubsystem(17);
        m_loadedMotor = new NeoMotorSubsystem(15);

        m_shooterMotorLeft = new TalonMotorSubsystem(7);
        m_shooterMotorRight = new TalonMotorSubsystem(12);

        // MOTOR COMMANDS //
        m_stagedMotorCommand = new NeoIndividualMotorCommand(m_stagedMotor, false);
        m_stagedMotorReversedCommand = new NeoIndividualMotorCommand(m_stagedMotor, true);
        m_ejectionMotorCommand = new NeoIndividualMotorCommand(m_ejectionMotor, false);
        m_ejectionMotorReversedCommand = new NeoIndividualMotorCommand(m_ejectionMotor, true);
        m_cargoCenteringMotorCommand = new NeoIndividualMotorCommand(m_cargoCenteringMotor, false);
        m_cargoCenteringMotorReversedCommand = new NeoIndividualMotorCommand(m_cargoCenteringMotor, true);
        m_intakeRollerMotorCommand = new NeoIndividualMotorCommand(m_intakeRollerMotor, false);
        m_intakeRollerMotorReversedCommand = new NeoIndividualMotorCommand(m_intakeRollerMotor, true);
        m_loadedMotorCommand = new NeoIndividualMotorCommand(m_loadedMotor, false);
        m_loadedMotorReversedCommand = new NeoIndividualMotorCommand(m_loadedMotor, true);

        m_shootMotorCommandLeft = new TalonIndividualMotorCommand(m_shooterMotorLeft, false);//12 and 7
        m_shootMotorCommandRight = new TalonIndividualMotorCommand(m_shooterMotorRight, true);

        // // INTAKE INITS //
        // m_IntakeSubsystem = new IntakeSubsystem(2, 17, 3);
        // m_RunIntakeRollersCommand = new RunIntakeRollersCommand(m_IntakeSubsystem);
        // m_ExtendIntakeCommand = new ExtendIntakeCommand(m_IntakeSubsystem);
        
        // // INDEXER INITS //
        // m_IndexerSubsystem = new IndexerSubsystem(16, 4);
        // m_IndexerSensorUtility = new IndexerSensorUtility(15, 16);
        // m_IndexerCommand = new IndexerCommand(m_IndexerSubsystem, m_IndexerSensorUtility, 0.5);
        // m_IndexerEjectCommand = new IndexerEjectCommand(m_IndexerSubsystem, 0.5);

        // //SHOOTER INITS //
        // m_ShooterMotorSubsystem = new ShooterMotorSubsystem(12, 7, 15);
        // m_ShooterHoodSubsystem = new ShooterHoodSubsystem(6);
        // m_ShooterCommand  = new ShooterCommand(m_ShooterMotorSubsystem, m_IndexerSubsystem, 0.5);
        // m_ShooterHoodCommand = new AdjustHoodCommand(m_ShooterHoodSubsystem);

        boolean fieldRelative = true;
        boolean openLoop = true;
        m_Swerve.setDefaultCommand(new TeleopSwerve(m_Swerve, m_driverController.getController(), translationAxis,
                strafeAxis, rotationAxis, fieldRelative, openLoop));

        compressor.enableAnalog(100, 115);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick
     * } or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        
        /* Driver Buttons */
        // Put back in after testing

        // m_driverController.getRightBumper().whileActiveOnce(
        //         m_ShooterCommand);

        m_driverController.getYButton().whileActiveOnce(
                new InstantCommand(() -> m_Swerve.zeroGyro()));
        
                
        // m_driverController.getYButton().whileActiveOnce(
        //         m_stagedMotorCommand);
                  
        // m_driverController.getAButton().whileActiveOnce(
        //         m_stagedMotorReversedCommand);
         
        // m_driverController.getXButton().whileActiveOnce(
        //         m_ejectionMotorCommand);
                 
        m_driverController.getBButton().whileActiveOnce(
                m_ejectionMotorReversedCommand);

        m_driverController.getRightBumper().whileActiveOnce(
                m_cargoCenteringMotorReversedCommand);

        m_driverController.getYButton().whileActiveOnce(
                m_intakeRollerMotorCommand);
        m_driverController.getPOVUpTrigger().whileActiveOnce(
            m_loadedMotorReversedCommand);
        m_driverController.getLeftBumper().whileActiveOnce(
            m_shootMotorCommandLeft);
        m_driverController.getLeftBumper().whileActiveOnce(
            m_shootMotorCommandRight);
        /* Co-Driver Buttons */

        // m_codriverController.getLeftBumper().whileActiveOnce(
        //         new ParallelCommandGroup(
        //                 m_IndexerCommand,
        //                 m_RunIntakeRollersCommand,
        //                 m_ExtendIntakeCommand));

        // m_codriverController.getBButton().whileActiveOnce(
        //         m_IndexerEjectCommand);

        // Tarmac
//         m_codriverController.getPOVLeftTrigger().whileActiveOnce(
//                 new ParallelCommandGroup(
//                         new AdjustHoodCommand(
//                                 m_ShooterHoodSubsystem,
//                                 ShooterUtility.calculateHoodPos(9)),
//                         new ShooterCommand(
//                                 m_ShooterMotorSubsystem,
//                                 m_IndexerSubsystem,
//                                 ShooterUtility.calculateTopSpeed(9),
//                                 ShooterUtility.calculateBottomSpeed(9))).withTimeout(0.1));

//         // Launchpad
//         m_codriverController.getPOVUpTrigger().whileActiveOnce(
//                 new ParallelCommandGroup(
//                         new AdjustHoodCommand(
//                                 m_ShooterHoodSubsystem,
//                                 ShooterUtility.calculateHoodPos(14.5)),
//                         new ShooterCommand(
//                                 m_ShooterMotorSubsystem,
//                                 m_IndexerSubsystem,
//                                 ShooterUtility.calculateTopSpeed(14.5),
//                                 ShooterUtility.calculateBottomSpeed(14.5))).withTimeout(0.1));

//         // Fender shot
//         m_codriverController.getPOVDownTrigger().whileActiveOnce(
//                 new ParallelCommandGroup(
//                         new AdjustHoodCommand(
//                                 m_ShooterHoodSubsystem,
//                                 ShooterUtility.calculateHoodPos(19 / 12)),
//                         new ShooterCommand(
//                                 m_ShooterMotorSubsystem,
//                                 m_IndexerSubsystem,
//                                 ShooterUtility.calculateTopSpeed(19 / 12),
//                                 ShooterUtility.calculateBottomSpeed(19 / 12))).withTimeout(0.1));

    }

  public void testExit() {
    //m_ShooterMotorSubsystem.refollowShooterMotors(); // TODO: Uncomment
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
