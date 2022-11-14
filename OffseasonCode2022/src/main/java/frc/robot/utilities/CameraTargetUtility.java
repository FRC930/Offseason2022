package frc.robot.utilities;

import java.util.HashMap;
import java.util.Map;

//  public class interface
public class CameraTargetUtility {

    //--------------------------------------------------------------------------
    // private instance
    private static CameraTargetUtility vInstance ;

    //  Map to hold all absolute postion for every target
    private Map<Integer, TargetInfo> m_Targets ;

    //--Constructor--//
    //  must be private so that no one can construct one of these objects
    private CameraTargetUtility() {
        //  Initialize Map
        m_Targets = new HashMap<Integer, TargetInfo>() ;
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