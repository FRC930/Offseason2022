// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoCommandManager.subNames;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.commands.autovisioncommands.PhotonAimCommand;
import frc.robot.commands.shooterCommands.AdjustHoodCommand;
import frc.robot.commands.shooterCommands.ShooterCommand;
import frc.robot.subsystems.*;
import frc.robot.utilities.PhotonVisionUtility;
import frc.robot.utilities.AprilVisionUtility;
import frc.robot.utilities.CameraTargetUtility;
import frc.robot.utilities.ShooterUtility;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;
import frc.lib.util.SwerveModuleConstants;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.net.PortForwarder;
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

        private PhotonCamera m_hubCamera;

        /* Constants */
        double IndexerMotorSpeed = 0.5; // TODO 0.5;

        /* Controllers */
        // Driver Controller
        private final CommandXboxController m_driverController = new CommandXboxController(0);
        // Codriver Controller
        private final CommandXboxController m_codriverController = new CommandXboxController(1);

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
        private final IntakeExtensionSubsystem m_IntakeExtention = new IntakeExtensionSubsystem(2);
        private final IntakeVoltageSubsystem m_IntakeVoltage = new IntakeVoltageSubsystem(2);
        private final IntakeSubsystem  m_IntakeSubsystem = new IntakeSubsystem(17, 3);
        private final ShooterMotorSubsystem m_ShooterMotorSubsystem = new ShooterMotorSubsystem(12, 7);
        private final ShooterHoodSubsystem m_ShooterHoodSubsystem = new ShooterHoodSubsystem(6);
        private final IndexerSubsystem  m_IndexerSubsystem = new IndexerSubsystem(16, 4);
        private final LoadedMotorSubsystem m_LoadedMotorSubsystem = new LoadedMotorSubsystem(15);


        NeoMotorSubsystem m_stagedMotor;
        NeoMotorSubsystem m_ejectionMotor;
        NeoMotorSubsystem m_cargoCenteringMotor;
        NeoMotorSubsystem m_intakeRollerMotor;
        NeoMotorSubsystem m_loadedMotor;

        TalonMotorSubsystem m_shooterMotorLeft;
        TalonMotorSubsystem m_shooterMotorRight;

        /* Commands */
        private  ExtendIntakeCommand m_ExtendIntakeCommand;
        private  RunIntakeRollersCommand m_RunIntakeRollersCommand;
        private PhotonAimCommand m_PhotonAimCommand;
        private final IndexerEjectCommand m_IndexerEjectCommand;
         private final ShooterCommand m_ShooterCommand;
         private final AdjustHoodCommand m_ShooterHoodCommand;


        TalonIndividualMotorCommand m_shootMotorCommandLeft;
        TalonIndividualMotorCommand m_shootMotorCommandRight;

        /* Utilities */
        private final IndexerSensorUtility m_IndexerSensorUtility;

        private final PhotonVisionUtility m_PhotonVisionUtility = PhotonVisionUtility.getInstance();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                // PORT FORWARDING //
                PortForwarder.add(5800, "10.9.30.25", 5800);
                PortForwarder.add(1181, "10.9.30.25", 1181);
                PortForwarder.add(1182, "10.9.30.25", 1182);
                PortForwarder.add(1183, "10.9.30.25", 1183);
                PortForwarder.add(1184, "10.9.30.25", 1184);

                /**
                 * The container for the robot. Contains subsystems, OI devices, and commands.
                 */



                // Auto Command Manager Stuff
                m_autoManager = new AutoCommandManager();
                m_autoManager.addSubsystem(subNames.Swerve, m_Swerve);
                m_autoManager.initCommands();

                // INDEXER INITS //
                m_IndexerSensorUtility = new IndexerSensorUtility(15, 16);
                m_IndexerEjectCommand = new IndexerEjectCommand(m_IndexerSubsystem, IndexerMotorSpeed);

                // MOTORS //
                /*
                m_stagedMotor = new NeoMotorSubsystem(16);
                m_ejectionMotor = new NeoMotorSubsystem(4);
                m_cargoCenteringMotor = new NeoMotorSubsystem(3);
                m_intakeRollerMotor = new NeoMotorSubsystem(17);
                m_loadedMotor = new NeoMotorSubsystem(15);

                m_shooterMotorLeft = new TalonMotorSubsystem(7);
                m_shooterMotorRight = new TalonMotorSubsystem(12);
                */

                //
                //      for testing purpose...

                // MOTOR COMMANDS //
                /*
                 * m_stagedMotorCommand = new NeoIndividualMotorCommand(m_stagedMotor, false);
                 * m_stagedMotorReversedCommand = new NeoIndividualMotorCommand(m_stagedMotor,
                 * true);
                 * m_ejectionMotorCommand = new NeoIndividualMotorCommand(m_ejectionMotor,
                 * false);
                 * m_ejectionMotorReversedCommand = new
                 * NeoIndividualMotorCommand(m_ejectionMotor, true);
                 * m_cargoCenteringMotorReversedCommand = new
                 * NeoIndividualMotorCommand(m_cargoCenteringMotor, true);
                 * m_intakeRollerMotorCommand = new
                 * NeoIndividualMotorCommand(m_intakeRollerMotor, false);
                 * m_loadedMotorReversedCommand = new NeoIndividualMotorCommand(m_loadedMotor,
                 * true);
                 */
                m_shootMotorCommandLeft = new TalonIndividualMotorCommand(m_shooterMotorLeft, false);
                m_shootMotorCommandRight = new TalonIndividualMotorCommand(m_shooterMotorRight, true);

                // // INTAKE INITS //


                // // INDEXER INITS //

                // m_IndexerCommand = new IndexerCommand(m_IndexerSubsystem,
                // m_IndexerSensorUtility, 0.5);
                // m_IndexerEjectCommand = new IndexerEjectCommand(m_IndexerSubsystem, 0.5);

                // //SHOOTER INITS //
                m_ShooterCommand = new ShooterCommand(m_ShooterMotorSubsystem, 0.5);
                m_ShooterHoodCommand = new AdjustHoodCommand(m_ShooterHoodSubsystem);
                m_PhotonAimCommand = new PhotonAimCommand(m_Swerve, m_driverController,  m_codriverController, m_hubCamera);

                boolean fieldRelative = true;
                boolean openLoop = true;
                m_Swerve.setDefaultCommand(new TeleopSwerve(m_Swerve, m_driverController, translationAxis,
                                                strafeAxis, rotationAxis, fieldRelative, openLoop));
                m_IntakeVoltage.setDefaultCommand(new IntakeStateCommand(m_IntakeVoltage, true));
        

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

                // TODO: make these both into a parrallel command group
                 m_driverController.rightBumper().whileTrue(new ExtendIntakeCommand(m_IntakeExtention));
                 m_driverController.rightBumper().whileTrue(new IntakeStateCommand(m_IntakeVoltage, false));

                 //m_driverController.get
                 //m_ShooterCommand);
                 m_driverController.leftBumper().whileTrue(           
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        m_PhotonAimCommand, 
                                        m_ShooterHoodCommand,
                                        m_ShooterCommand),
                                new RunIndexerCommand(m_IndexerSubsystem, m_LoadedMotorSubsystem, IndexerMotorSpeed) 
                        )
                        
                );

                /* Co-Driver Buttons */

                /*m_codriverController.getLeftBumper().whileActiveOnce(
                                new ParallelCommandGroup(
                                                m_IndexerCommand,
                                                m_RunIntakeRollersCommand,
                                                m_ExtendIntakeCommand));
                */
                m_codriverController.b().whileTrue(
                m_IndexerEjectCommand);

                // Tarmac
                m_codriverController.povLeft().whileTrue(
                new AdjustHoodCommand(
                m_ShooterHoodSubsystem,
                ShooterUtility.calculateHoodPos(9))
                );

                // // Launchpad
                m_codriverController.povUp().whileTrue(
                        new AdjustHoodCommand(
                                m_ShooterHoodSubsystem,
                                ShooterUtility.calculateHoodPos(14.5)));

                // // Fender shot
                m_codriverController.povDown().whileTrue(
                new AdjustHoodCommand(
                m_ShooterHoodSubsystem,
                ShooterUtility.calculateHoodPos(19 / 12)
                ));

        }

        public void testExit() {
                // m_ShooterMotorSubsystem.refollowShooterMotors(); // TODO: Uncomment
                m_IntakeSubsystem.followIntakeMotors();
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
