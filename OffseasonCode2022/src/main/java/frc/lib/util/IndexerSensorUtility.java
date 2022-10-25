package frc.lib.util;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;

public class IndexerSensorUtility {

    //-------- CONSTANTS --------\\

    private final int TRIGGER_DISTANCE = 200;
    
    //-------- VARIABLES --------\\
    
    private DigitalInput loadedSensorSim;
    private DigitalInput stagedSensorSim;

    private TimeOfFlight loadedSensor;
    private TimeOfFlight stagedSensor;

    //-------- CONSTRUCTOR --------\\
    /**
     * <h3>EndgameSensorUtility</h3>
     * 
     * Creates and returns sensor values the endgame sensors
     */
    public IndexerSensorUtility(int loadedId, int stagedId) {
        if (Robot.isReal()) {
            loadedSensor = new TimeOfFlight(loadedId);
            stagedSensor = new TimeOfFlight(stagedId);
            loadedSensor.setRangingMode(RangingMode.Short, 25);
            stagedSensor.setRangingMode(RangingMode.Short, 25);
        } else {
            loadedSensorSim = new DigitalInput(loadedId);
            stagedSensorSim = new DigitalInput(stagedId);
        }
    }

    //-------- METHODS --------\\

    /**
     * NOTE: DIO is opposite of sensor contact
     * DIO is 0 when sensor is activated(touching metal)
     */

    /**
     * <h3>loadedIsTouching</h3>
     * 
     * This method returns the loaded sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean loadedIsTouching() {
        if (Robot.isReal()) {
            return loadedSensor.getRange() < TRIGGER_DISTANCE;
        } else {
            return loadedSensorSim.get();
        }
    }

    /**
     * <h3>stagedIsTouching</h3>
     * 
     * This method returns the staged sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean stagedIsTouching() {
        if (Robot.isReal()) {
            return stagedSensor.getRange() < TRIGGER_DISTANCE;
        } else {
            return stagedSensorSim.get();
        }
    }

} // End of class IndexerSensorUtility
