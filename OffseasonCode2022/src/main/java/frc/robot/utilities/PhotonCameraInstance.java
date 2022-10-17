package frc.robot.utilities;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

public class PhotonCameraInstance {
    private PhotonCamera m_PhotonCamera  ;

    //
    //  const transform for center of bot
    final Pose2d m_RobotCenter = new Pose2d(0.0, 0.0, new Rotation2d(0.0)) ;
    
    //
    //  relative postion of camera to center of bot
    //  -- current example is .5 meters in the x and y direction, with the direction of camer 180 deg -> 3.14159 radians from robot forward
    final Pose2d m_RelCamPosFromRobotCenter = new Pose2d(0.5, 0.5, new Rotation2d(3.14159)) ;

    //
    //  create constant camra transform from robot center
    private Transform2d m_CameraPosition = new Transform2d(m_RobotCenter, m_RelCamPosFromRobotCenter) ;

    public PhotonCameraInstance (String vCameraName, int vPipeLineIndex) {
        //  setup the camera obj
        m_PhotonCamera = new PhotonCamera(vCameraName) ;
        m_PhotonCamera.setPipelineIndex(vPipeLineIndex) ;
        m_PhotonCamera.setDriverMode(false) ;

        // Set the banner to show the driver that photon isn't ready yet
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                ShuffleboardKeys.PHOTON_READY, new ShuffleBoardData<Boolean>(false));
    }

    public void SetDriveMode(boolean vEnable) {
        m_PhotonCamera.setDriverMode(vEnable) ;
    }

    public Pose2d CalculatePosition() {
        // grab camera result
        PhotonPipelineResult vResult = m_PhotonCamera.getLatestResult() ;
        Pose2d vRotationRange = new Pose2d() ;
        
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
            vRotationRange = PhotonUtils.estimateFieldToRobot(vTargetTranform, CameraTargetUtility.getInstance().getTarget(vTarget.getFiducialId()).getTargetPos(), m_CameraPosition) ;
        }

        return vRotationRange ;
    }
}
