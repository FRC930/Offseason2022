package frc.robot.utilities;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.Swerve;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

public class PhotonCameraInstance {
    //
    //  photon camera instance
    private PhotonCamera m_PhotonCamera  ;

    //
    //  swerve drive subsystem
    //  -- need this to get gyro information
    private Swerve m_DrivetrainSubsystem ;

    //
    //  serve drive position estimator
    private SwerveDrivePoseEstimator m_PoseEstimator ;

    //
    //  Kalma Filter Configuration.
    //  -- These matrix's are used to add or subtracka constant deviation from what is read from camer/robot
    //  -- Change the values to tweek data retrieved from pose estimator
    private final Matrix<N3, N1> m_StateStdDevs = VecBuilder.fill(0.0, 0.0, Units.degreesToRadians(0));
    private final Matrix<N1, N1> m_LocalMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.0));
    private final Matrix<N3, N1> m_VisionMeasurementStdDevs = VecBuilder.fill(0.0, 0.0, Units.degreesToRadians(0));

    //
    //  relative postion of camera to center of bot
    //  -- current example is 10 inch in the x and y direction, with the direction of camer 180 deg -> 3.14159 radians from robot forward
    private final Transform2d m_CameraPosition = new Transform2d(new Translation2d(Units.inchesToMeters(10), 0.0), new Rotation2d(3.14159)) ;

    //-- Constructor --//
    public PhotonCameraInstance (String vCameraName, int vPipeLineIndex, Swerve vDriveTrainSubsystem) {
        //  setup the camera obj
        m_PhotonCamera = new PhotonCamera(vCameraName) ;
        m_PhotonCamera.setPipelineIndex(vPipeLineIndex) ;
        m_PhotonCamera.setDriverMode(false) ;

        //
        //  setup the swerve drive subsystem
        m_DrivetrainSubsystem = vDriveTrainSubsystem ;

        //
        //  setup the serve pose estimator
        m_PoseEstimator = new SwerveDrivePoseEstimator(m_DrivetrainSubsystem.getYaw(), new Pose2d(), m_DrivetrainSubsystem.swerveKinematics, m_StateStdDevs, m_LocalMeasurementStdDevs, m_VisionMeasurementStdDevs) ;

        // Set the banner to show the driver that photon isn't ready yet
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                ShuffleboardKeys.PHOTON_READY, new ShuffleBoardData<Boolean>(false));
    }

    public void SetDriveMode(boolean vEnable) {
        m_PhotonCamera.setDriverMode(vEnable) ;
    }

    public Pose2d CalculatePosition() {
        //  initialize return
        Pose2d vPose = new Pose2d() ;
        
        /* ---- Using PhotonUtils to estimate field position
        // grab camera result
        PhotonPipelineResult vResult = m_PhotonCamera.getLatestResult() ;
        
        //  check if camera has a result
        if(vResult.hasTargets()) {
            //  grab the best target
            PhotonTrackedTarget vTarget = vResult.getBestTarget() ;

            //  grab transform/rotation from target
            Translation2d vTargetTranslation = vTarget.getCameraToTarget().getTranslation().toTranslation2d() ;
            Rotation2d vTargetRotation = vTarget.getCameraToTarget().getRotation().toRotation2d() ;

            //  combine into a Transform2D for use in estimater
            Transform2d vTargetTranform = new Transform2d(vTargetTranslation, vTargetRotation) ;

            //  apply target transform3d
            //  --grab target ID for referencing target absolute position
            //  -- like to see difference in values between this calculation and swervePosEstimator
            vPose = PhotonUtils.estimateFieldToRobot(vTargetTranform, CameraTargetUtility.getInstance().getTarget(vTarget.getFiducialId()).getTargetPos(), m_CameraPosition) ;
        }*/

        // --- use swerve pose estimator to determine robot position
        vPose = m_PoseEstimator.getEstimatedPosition() ;

        return vPose ;
    }

    public void UpdateCameraPos() {
        // grab camera result
        PhotonPipelineResult vResult = m_PhotonCamera.getLatestResult() ;
        
        //  check if camera has a result
        if(vResult.hasTargets()) {
            //  grab the best target
            PhotonTrackedTarget vTarget = vResult.getBestTarget() ;
            int vTargetID = vTarget.getFiducialId() ;

            //  grab transform/rotation from target
            Translation2d vTargetTranslation = vTarget.getCameraToTarget().getTranslation().toTranslation2d() ;
            Rotation2d vTargetRotation = vTarget.getCameraToTarget().getRotation().toRotation2d() ;

            //  combine into a Transform2D for use in estimater
            Transform2d vTargetTranform = new Transform2d(vTargetTranslation, vTargetRotation) ;

            //
            //  calculate pose2d from swerve pos estimator
            Pose2d vInvertCamPose = CameraTargetUtility.getInstance().getTarget(vTargetID).getTargetPos().transformBy(vTargetTranform.inverse()) ;
            Pose2d vVisionMeasurment = vInvertCamPose.transformBy(m_CameraPosition) ;
            double vImageCaptureTime = Timer.getFPGATimestamp() - (vResult.getLatencyMillis() / 1000d) ;
            m_PoseEstimator.addVisionMeasurement(vVisionMeasurment, vImageCaptureTime) ;
        }

        //
        //  update the pos estimator
        m_PoseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_DrivetrainSubsystem.getYaw(), m_DrivetrainSubsystem.getStates()) ;
    }
}
