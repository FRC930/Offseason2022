package frc.robot.subsystems;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.text.ParseException;
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
  private JSONParser m_parser ;
  JSONArray m_TagList;
  Iterator<JSONObject> m_Iterator;

  public CameraSubsystem(PhotonCameraInstance vCameraInstance) {
      m_cameraInstance = vCameraInstance ;
      m_shuffleboardUtil = ShuffleboardUtility.getInstance() ;
      m_parser = new JSONParser();
      try{
        JSONObject vobj = (JSONObject) m_parser.parse(new FileReader(new File(Filesystem.getDeployDirectory(), "AprilTagInfo/TagLocations.json")));
        m_TagList = (JSONArray) vobj.get("tagList");
        m_Iterator = m_TagList.iterator();

        //
        //  read file
        while (m_Iterator.hasNext())
        {
            CameraTargetUtility.getInstance().addTarget(Integer.parseInt(m_Iterator.next().get("tagID").toString()), new TargetInfo(m_Iterator.next().get("tagLabel").toString(), new Pose2d(Double.parseDouble(m_Iterator.next().get("xPos").toString()), Double.parseDouble(m_Iterator.next().get("yPos").toString()), new Rotation2d(0.0))));
        }
      } catch (Exception e) {
        e.printStackTrace();
      }
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
