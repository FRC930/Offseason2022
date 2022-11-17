// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.AutoCommandManager.subNames;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.commands.autovisioncommands.PhotonAimCommand;
import frc.robot.commands.shooterCommands.AdjustHoodCommand;
import frc.robot.commands.shooterCommands.ShooterCommand;
import frc.robot.subsystems.*;
import frc.robot.utilities.CameraTargetUtility;
import frc.robot.utilities.PhotonCameraInstance;
import frc.robot.utilities.PhotonVisionUtility;
import frc.robot.utilities.ShooterUtility;
import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.util.net.PortForwarder;
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

    /* Constants */
    double IndexerMotorSpeed = 0.5; // TODO 0.5;

    /* Utilities */
    private final PhotonVisionUtility m_PhotonVisionUtility = PhotonVisionUtility.getInstance();

    /* Shuffleboard */
    private final ShuffleboardSubsystem m_shuffleboardSubsystem = new ShuffleboardSubsystem() ;

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
    private final IndexerSensorUtility m_IndexerSensorUtility = new IndexerSensorUtility(15, 6);

    /* Auto Command Manager */
    private final AutoCommandManager m_autoManager = new AutoCommandManager() ;

    /* Modules */
    private static final SwerveModuleConstants frontLeftModule = new SwerveModuleConstants(8, 9, 9, 259.980);
    private static final SwerveModuleConstants frontRightModule = new SwerveModuleConstants(11, 10, 10, 233.877);
    private static final SwerveModuleConstants backLeftModule = new SwerveModuleConstants(1, 0, 0, 71.895);
    private static final SwerveModuleConstants backRightModule = new SwerveModuleConstants(18, 19, 19, 143.965);

    /* Subsystems */
    private final Swerve m_Swerve = new Swerve(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem(17, 3);
    private final ShooterMotorSubsystem m_ShooterMotorSubsystem = new ShooterMotorSubsystem(12, 7);
    private final ShooterHoodSubsystem m_ShooterHoodSubsystem = new ShooterHoodSubsystem(6);
    private final IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem(16, 4);
    private final IntakeVoltageSubsystem m_IntakeVoltage = new IntakeVoltageSubsystem(2);
    private final LoadedMotorSubsystem m_LoadedMotorSubsystem = new LoadedMotorSubsystem(15);

    /* Camera instance */
    //  initiate the utility here to instantiate the resource for whole project
    //  -- create a subsystem to run the camera udate position through periodic of subsystem
    private final PhotonCameraInstance m_PhotonCamer = new PhotonCameraInstance("PiCamera", 0, m_Swerve) ;
    private final CameraTargetUtility m_CameraTargetUtil = CameraTargetUtility.getInstance() ;
    private final CameraSubsystem m_CameraSubstem= new CameraSubsystem(m_PhotonCamer) ;

    /** RobotContianer
     * The container for the robot. Contains subsystems, OI devices, and commands. 
    */
    public RobotContainer() {
        // PORT FORWARDING //
        PortForwarder.add(5800, "10.9.30.25", 5800);
        PortForwarder.add(1181, "10.9.30.25", 1181);
        PortForwarder.add(1182, "10.9.30.25", 1182);
        PortForwarder.add(1183, "10.9.30.25", 1183);
        PortForwarder.add(1184, "10.9.30.25", 1184);

        //  Auto Command Manager Stuff
        m_autoManager.addSubsystem(subNames.Swerve, m_Swerve);
        m_autoManager.initCommands();

        // Drive INITS //
        boolean fieldRelative = true;       
        boolean openLoop = true;
        m_Swerve.setDefaultCommand(new TeleopSwerve(m_Swerve, m_driverController.getController(), translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));
        m_IntakeVoltage.setDefaultCommand(new IntakeStateCommand(m_IntakeVoltage, true));
        m_CameraSubstem.setDefaultCommand(new CameraDefaultCommand(m_CameraSubstem)) ;

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
        m_driverController.getRightBumper().whileActiveOnce(           
            new SequentialCommandGroup(
                new ShooterCommand(m_ShooterMotorSubsystem),
                new WaitCommand(0.2),
                new RunIndexerCommand(m_IndexerSubsystem, m_LoadedMotorSubsystem, IndexerMotorSpeed)  
            )
                
        );
        
        m_driverController.getLeftBumper().whileActiveOnce(             
            new SequentialCommandGroup(
                new PhotonAimCommand(m_Swerve, m_CameraSubstem, m_driverController.getController(),  m_codriverController.getController()), 
                new AdjustHoodCommand(m_ShooterHoodSubsystem),
                new ShooterCommand(m_ShooterMotorSubsystem),
                new WaitCommand(0.2),
                new RunIndexerCommand(m_IndexerSubsystem, m_LoadedMotorSubsystem, IndexerMotorSpeed)
            )  
                
        );

        /* Co-Driver Buttons */

        // TODO: make a parallel command
        m_codriverController.getRightBumper().whileActiveOnce(new IndexerCommand(m_IndexerSubsystem, m_IndexerSensorUtility, m_LoadedMotorSubsystem, IndexerMotorSpeed));
        m_codriverController.getRightBumper().whileActiveOnce(new RunIntakeRollersCommand(m_IntakeSubsystem));
        m_codriverController.getRightBumper().whileActiveOnce(new IntakeStateCommand(m_IntakeVoltage, false));

        m_codriverController.getBButton().whileActiveOnce(new IndexerEjectCommand(m_IndexerSubsystem, IndexerMotorSpeed));

        // Tarmac
        m_codriverController.getPOVLeftTrigger().whileActiveOnce(getPovCommand(ShooterUtility.calculateHoodPos(7)));

        // // Launchpad
        m_codriverController.getPOVUpTrigger().whileActiveOnce(getPovCommand(ShooterUtility.calculateHoodPos(14.5)));

        // // Fender shot
        m_codriverController.getPOVDownTrigger().whileActiveOnce(getPovCommand(ShooterUtility.calculateHoodPos(19 / 12)));
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

  /**
     * <h3>beginTeleopRunCommands</h3>
     * 
     * Runs when the robot is enabled in teleop mode.
     * 
     * This gets the command scheduler and sets up buttons
     */
    public void beginTeleopRunCommands() {
        //  initialize stuff for teleop
    }

 /**
     * <h3>beginAutoRunCommands</h3>
     * 
     * Prepares the robot for autonomous.
     */
    public void beginAutoRunCommands() {
        //  initialize stuff for autonomous
    }

    /**
     * <h3>testInit</h3>
     * 
     * Initializes the robot for test mode.
     */
    public void testInit() {
        //  initialize test mode
    }

    /**
     * <h3>testPeriodic</h3>
     * 
     * Test mode periodic.
     */
    public void testPeriodic() {
        // TODO
        //  show a list of the target and be able to change the representatio for each target
    }

    /**
     * <h3>testExit</h3>
     * 
     * Runs when robot exits test mode.
     */
    public void testExit() {
        m_ShooterMotorSubsystem.refollowShooterMotors(); 
        m_IntakeSubsystem.followIntakeMotors();
    }

    /**
     * <h3>getPovCommand</h3>
     * 
     * Creates a command group for adjusting hood
     * @param distance
     * @return
     */
    private Command getPovCommand(double distance){
        Command cmd;
        cmd = new AdjustHoodCommand(m_ShooterHoodSubsystem, ShooterUtility.calculateHoodPos(distance));
        cmd = new ParallelCommandGroup(
            new SequentialCommandGroup( 
                new AdjustHoodCommand(m_ShooterHoodSubsystem, ShooterUtility.calculateHoodPos(distance)),
                new ShooterCommand(m_ShooterMotorSubsystem,ShooterUtility.calculateTopSpeed(distance))
            ),
            new RunIndexerCommand(m_IndexerSubsystem, m_LoadedMotorSubsystem, IndexerMotorSpeed)  
        );
        return cmd;
    }
}
