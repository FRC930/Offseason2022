//----- IMPORTS -----\\

package frc.robot.commands.autovisioncommands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.utilities.PhotonVisionUtility;
import frc.robot.utilities.ShooterUtility;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;
import frc.robot.utilities.VisionSmoothingStack;

//----- CLASS -----\\
/**
 * <h3>HubAimingCommand</h3>
 * 
 * Rotates the robot to aim at the cargo hub.
 */
public class PhotonAimCommand extends CommandBase {

    // ----- CONSTANTS -----\\

    // Target yaw offset in degrees
    public static final double YAW_DEADBAND = 2;

    // The height of the camera
    private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(34.0);
    // The height of the hub
    private final double HUB_HEIGHT_METERS = Units.inchesToMeters(104);

    // Get the difference in height between the camera and the hub
    private final double HEIGHT_DIFFERENCE_METERS = HUB_HEIGHT_METERS - CAMERA_HEIGHT_METERS;

    // The pitch of the camera (from ground normal)
    private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(36.0);

    private final double ANGULAR_P = 0.21;

    private final double ANGULAR_I = 0.01;
    private final double ANGULAR_D = 0.01;

    // ----- VARIABLES -----\\

    private PIDController m_turnController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

    protected PhotonCamera m_hubCamera = PhotonVisionUtility.getInstance().getHubTrackingCamera();
    private Swerve m_Swerve;
    private CameraSubsystem m_Camera;

    private int cyclesAimed = 0;

    private VisionSmoothingStack m_smoothingStack = new VisionSmoothingStack(3);

    private XboxController m_driverController;
    private XboxController m_codriverController;

    // ----- CONSTRUCTORS -----\\

    /**
     * <h3>HubAimingCommand</h3>
     * 
     * Rotates the robot to aim at the cargo hub.
     * 
     * @param dSwerve
     */
    public PhotonAimCommand(Swerve dSwerve, CameraSubsystem dCamera) {
        this(dSwerve, dCamera, null, null);
    }

    /**
     * <h3>HubAimingCommand</h3>
     * 
     * Rotates the robot to aim at the cargo hub.
     * 
     * @param dSwerve         the drive subsystem to use to rotate the robot
     * @param driverController   driver controller (for rumble)
     * @param codriverController driver controller (for rumble)
     */
    public PhotonAimCommand(Swerve dSwerve, CameraSubsystem dCamera, XboxController driverController,
            XboxController codriverController) {

        m_Swerve = dSwerve;
        m_Camera = dCamera;
        m_driverController = driverController;
        m_codriverController = codriverController;

        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                ShuffleboardKeys.AIMED, new ShuffleBoardData<Boolean>(false));

        //  not adding camera subsystem as this system is only being read from
        addRequirements(dSwerve);
    }

    // ----- METHODS -----\\

    @Override
    public void initialize() {

        // Set aimed to false to ensure that we do not exit right away
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                ShuffleboardKeys.AIMED, new ShuffleBoardData<Boolean>(false));

        // Set the pipeline on the camera to be whatever the user has selected on
        // shuffleboard
        //PhotonVisionUtilityUltimate.getInstance()
        //        .setPiCameraPipeline(ShuffleboardUtility.getInstance().getSelectedPipelineChooser());

        m_hubCamera.setLED(VisionLEDMode.kOn);
    }

    @Override
    public void execute() {
        // Variables to store our speeds
        double rotationSpeed;

        // Data that we get from the camera
        var result = m_hubCamera.getLatestResult();

        // If an item is detected
        if (result.hasTargets()) {
            // Finds the best target and puts into the stack
            m_smoothingStack.addItem(result.getBestTarget());

            ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                    ShuffleboardKeys.PHOTON_YAW, new ShuffleBoardData<Double>(m_smoothingStack.getAverageYaw()));

            // Use the PhotonUtils library to calcluate the distance from the target
            double range = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, HUB_HEIGHT_METERS,
                    CAMERA_PITCH_RADIANS, Units.degreesToRadians(m_smoothingStack.getAveragePitch()))
                    // Offset adjustment for distance math
                    + Units.inchesToMeters(3.5)
                    - 0.05;

            ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                    ShuffleboardKeys.DISTANCE_FROM_GOAL, new ShuffleBoardData<Double>(range));

            // Calculate shooter values
            ShooterUtility.setValuesToShuffleboard(Units.metersToFeet(range));

            // Use the turn PID controller to calculate how fast we want to turn
            rotationSpeed = m_turnController.calculate(m_smoothingStack.getAverageYaw(), 0);

            // Put if we are locked onto the target to the Shuffleboard
            if (Math.abs(m_smoothingStack.getAverageYaw()) < YAW_DEADBAND) {
                ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                        ShuffleboardKeys.AIMED, new ShuffleBoardData<Boolean>(true));

                cyclesAimed++;

                // Rumble both controllers
                if (m_driverController != null && m_codriverController != null) {
                    m_driverController.setRumble(RumbleType.kLeftRumble, 1);
                    m_driverController.setRumble(RumbleType.kRightRumble, 1);
                    m_codriverController.setRumble(RumbleType.kLeftRumble, 1);
                    m_codriverController.setRumble(RumbleType.kRightRumble, 1);
                }
            } else {
                ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                        ShuffleboardKeys.AIMED, new ShuffleBoardData<Boolean>(false));

                cyclesAimed = 0;

                // Turn off rumble for both controllers
                if (m_driverController != null && m_codriverController != null) {
                    m_driverController.setRumble(RumbleType.kLeftRumble, 0);
                    m_driverController.setRumble(RumbleType.kRightRumble, 0);
                    m_codriverController.setRumble(RumbleType.kLeftRumble, 0);
                    m_codriverController.setRumble(RumbleType.kRightRumble, 0);
                }
            }
        } else {
            rotationSpeed = 0.0;

            // Set shuffleboard distance to zero if no target
            ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                    ShuffleboardKeys.DISTANCE_FROM_GOAL, new ShuffleBoardData<Double>(0.0));

            // Set distance to shortest shot for shooter math
            ShooterUtility.setValuesToShuffleboard(1.6);
        }
        
        m_Swerve.drive(new Translation2d(0,0), rotationSpeed, true, true);
    }

    @Override
    public void end(boolean interrupted) {

        cyclesAimed = 0;

        // Tell shuffleboard utility that we are no longer aimed. This will make sure
        // that we don't think that we are aimed when we aren't
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                ShuffleboardKeys.AIMED, new ShuffleBoardData<Boolean>(false));

        // Turn off rumble for both controllers
        if (m_driverController != null && m_codriverController != null) {
            m_driverController.setRumble(RumbleType.kLeftRumble, 0);
            m_driverController.setRumble(RumbleType.kRightRumble, 0);
            m_codriverController.setRumble(RumbleType.kLeftRumble, 0);
            m_codriverController.setRumble(RumbleType.kRightRumble, 0);
        }

        m_hubCamera.setLED(VisionLEDMode.kOff);

        m_Swerve.drive(new Translation2d(0,0), 0.0, true, true);


    }

    @Override
    public boolean isFinished() {
        return cyclesAimed > 2;//4;
        //how many time is has to find a target
    }
}
