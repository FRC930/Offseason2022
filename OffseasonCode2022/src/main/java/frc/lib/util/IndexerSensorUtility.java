package frc.lib.util;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

public class IndexerSensorUtility {

    //-------- CONSTANTS --------\\

    private final int TRIGGER_DISTANCE = 200;
    
    //-------- VARIABLES --------\\
    
    private DigitalInput loadedSensorSim;
    private DigitalInput stagedSensorSim;

    private TimeOfFlight loadedSensor;
    private TimeOfFlight stagedSensor;

    private Debouncer m_Debouncer1;
    private Debouncer m_Debouncer2;

    //-------- CONSTRUCTOR --------\\
    /**
     * <h3>EndgameSensorUtility</h3>
     * 
     * Creates and returns sensor values the endgame sensors
     */
    public IndexerSensorUtility(int loadedId, int stagedId) {
        m_Debouncer1 = new Debouncer(0.5);
        m_Debouncer2 = new Debouncer(0.35);
        if (Robot.isReal()) {
            loadedSensor = new TimeOfFlight(loadedId);
            stagedSensor = new TimeOfFlight(stagedId);
            loadedSensor.setRangingMode(RangingMode.Medium, 20);
            stagedSensor.setRangingMode(RangingMode.Medium, 20);
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
            ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab, 
                                                                ShuffleboardKeys.LOADED_SENSOR, 
                                                                new ShuffleBoardData<Double>(loadedSensor.getRange()));
            return m_Debouncer1.calculate(loadedSensor.getRange() < TRIGGER_DISTANCE);
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
            ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab, 
                                                                ShuffleboardKeys.STAGED_SENSOR, 
                                                                new ShuffleBoardData<Double>(stagedSensor.getRange()));
            return m_Debouncer2.calculate(stagedSensor.getRange() < TRIGGER_DISTANCE);
        } else {
            return stagedSensorSim.get();
        }
    }

} // End of class IndexerSensorUtility
