package frc.lib.util;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;

public class IndexerSensorUtility {

    //-------- CONSTANTS --------\\

    private final int TRIGGER_DISTANCE = 200;
    
    //-------- VARIABLES --------\\
    
    private DigitalInput intakeSensorSim;
    private DigitalInput loadedSensorSim;
    private DigitalInput stagedSensorSim;
    private DigitalInput ejectionSensorSim;

    private TimeOfFlight intakeSensor;
    private TimeOfFlight loadedSensor;
    private TimeOfFlight stagedSensor;
    private TimeOfFlight ejectionSensor;

    //-------- CONSTRUCTOR --------\\
    /**
     * <h3>EndgameSensorUtility</h3>
     * 
     * Creates and returns sensor values the endgame sensors
     */
    public IndexerSensorUtility(int intakeId, int loadedId, int stagedId, int ejectionId) {
        if (Robot.isReal()) {
            intakeSensor = new TimeOfFlight(intakeId);
            loadedSensor = new TimeOfFlight(loadedId);
            stagedSensor = new TimeOfFlight(stagedId);
            ejectionSensor = new TimeOfFlight(ejectionId);
            intakeSensor.setRangingMode(RangingMode.Short, 25);
            loadedSensor.setRangingMode(RangingMode.Short, 25);
            stagedSensor.setRangingMode(RangingMode.Short, 25);
            ejectionSensor.setRangingMode(RangingMode.Short, 25);
        } else {
            intakeSensorSim = new DigitalInput(intakeId);
            loadedSensorSim = new DigitalInput(loadedId);
            stagedSensorSim = new DigitalInput(stagedId);
            ejectionSensorSim = new DigitalInput(ejectionId);
        }
    }

    //-------- METHODS --------\\

    /**
     * NOTE: DIO is opposite of sensor contact
     * DIO is 0 when sensor is activated(touching metal)
     */

    /**
     * <h3>intakeIsTouching</h3>
     * 
     * This method returns the intake sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean intakeIsTouching() {
        if (Robot.isReal()) {
            return intakeSensor.getRange() < TRIGGER_DISTANCE;
        } else {
            return intakeSensorSim.get();
        }
    }

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

    /**
     * <h3>ejectionIsTouching</h3>
     * 
     * This method returns the ejection sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean ejectionIsTouching() {
        if (Robot.isReal()) {
            return ejectionSensor.getRange() < TRIGGER_DISTANCE;
        } else {
            return ejectionSensorSim.get();
        }
    }

} // End of class IndexerSensorUtility
