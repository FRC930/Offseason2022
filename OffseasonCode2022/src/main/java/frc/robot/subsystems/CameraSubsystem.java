package frc.robot.subsystems;

import java.io.File;
import java.io.FileReader;
import java.util.Iterator;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.CameraTargetUtility;
import frc.robot.utilities.PhotonCameraInstance;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.TargetInfo;

public class CameraSubsystem extends SubsystemBase {
  private PhotonCameraInstance m_cameraInstance ;
  private ShuffleboardUtility m_shuffleboardUtil ;

  public CameraSubsystem(PhotonCameraInstance vCameraInstance) {
      m_cameraInstance = vCameraInstance ;
      m_shuffleboardUtil = ShuffleboardUtility.getInstance() ;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_cameraInstance.UpdateCameraPos() ;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Pose2d getRobotPosition() {
    return m_cameraInstance.CalculatePosition() ;
  }
}
