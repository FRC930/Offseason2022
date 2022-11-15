package frc.robot.utilities;

import java.io.File;
import java.io.FileReader;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;

//  public class interface
//  -- this class is used to store the possible targets for april tags
//  -- these tags will be stored in a map for easy reference
public class CameraTargetUtility {

    //--------------------------------------------------------------------------
    // private instance
    private static CameraTargetUtility vInstance ;
    
    //
    //  shuffleboard utility instance
    private ShuffleboardUtility m_shuffleboardUtil ;

    //  Map to hold all absolute postion for every target
    private Map<Integer, TargetInfo> m_Targets ;

    private JSONParser m_parser ;
    private JSONArray m_TagList;
    private Iterator<JSONObject> m_Iterator;

    //--Constructor--//
    //  must be private so that no one can construct one of these objects
    private CameraTargetUtility() {
        //  Initialize Map
        m_Targets = new HashMap<Integer, TargetInfo>() ;
        m_shuffleboardUtil = ShuffleboardUtility.getInstance();
        m_parser = new JSONParser();
      try{
        JSONObject vobj = (JSONObject) m_parser.parse(new FileReader(new File(Filesystem.getDeployDirectory(), "AprilTagInfo/TagLocations.json")));
        m_TagList = (JSONArray) vobj.get("tagList");
        m_Iterator = m_TagList.iterator();

        //
        //  read file
        while (m_Iterator.hasNext())
        {
            JSONObject vThisObj = (JSONObject) m_Iterator.next();
            int vThisTagID = Integer.parseInt(vThisObj.get("tagID").toString());
            String vThisTagLabel = vThisObj.get("tagLabel").toString();
            Double vThisXPos = Double.parseDouble(vThisObj.get("xPos").toString());
            Double vThisYPos = Double.parseDouble(vThisObj.get("yPos").toString());
            TargetInfo vThisInfo = new TargetInfo(vThisTagLabel, new Pose2d(vThisXPos, vThisYPos, new Rotation2d(0.0)));
            addTarget(vThisTagID, vThisInfo);
            m_shuffleboardUtil.putToShuffleboard(ShuffleboardUtility.testingTab, vThisTagLabel.concat("_ID"), new ShuffleBoardData<Integer>(vThisTagID));
            m_shuffleboardUtil.putToShuffleboard(ShuffleboardUtility.testingTab, vThisTagLabel.concat("_XPos"), new ShuffleBoardData<Double>(vThisXPos));
            m_shuffleboardUtil.putToShuffleboard(ShuffleboardUtility.testingTab, vThisTagLabel.concat("_YPos"), new ShuffleBoardData<Double>(vThisYPos));
        }
      } catch (Exception e) {
        e.printStackTrace();
      }
    }

    /**
     * <h3>getInstance</h3>
     * 
     * ShuffleboardUtility is a singleton, so getInstance returns the instance of
     * the class that the program will use
     * 
     * @return the instance
     */
    public static CameraTargetUtility getInstance() {
        if (vInstance == null) {
            vInstance = new CameraTargetUtility();
        }
        return vInstance;
    }

    /**
     * <h3>getGoalTarget</h3>
     * 
     * returns the position on the field for the active goal
     * 
     * @return Pose2D
     */
    public TargetInfo getTarget(Integer vID) {
        return m_Targets.get(vID) ;
    }

    /**
     * <h3>addTarget</h3>
     * 
     * Adds a new possible target
     * 
     */
    public void addTarget(Integer vID, TargetInfo vInfo) {
        m_Targets.put(vID, vInfo) ;
    }

    /**
     * <h3>hasTarget</h3>
     * 
     * does this target exist in map already
     * 
     */
    public boolean targetExists(Integer vID) {
        return (null != m_Targets.get(vID)) ;
    }
}