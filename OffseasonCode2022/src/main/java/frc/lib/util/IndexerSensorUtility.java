package frc.lib.util;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;

public class IndexerSensorUtility {
    
    //-------- CONSTANTS --------\\
    //-------- VARIABLES --------\\
    
    private DigitalInput intakeSensor;
    private DigitalInput loadedSensor;
    private DigitalInput stagedSensor;
    private DigitalInput ejectionSensor;

    //-------- CONSTRUCTOR --------\\
    /**
     * <h3>EndgameSensorUtility</h3>
     * 
     * Creates and returns sensor values the endgame sensors
     */
    public IndexerSensorUtility(int Intake_ID, int Loaded_ID, int Staged_ID, int Ejection_ID) {
        if (Robot.isReal()) {

        } else {
            intakeSensor = new DigitalInput(Intake_ID);
            loadedSensor = new DigitalInput(Loaded_ID);
            stagedSensor = new DigitalInput(Staged_ID);
            ejectionSensor = new DigitalInput(Ejection_ID);
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
        return !intakeSensor.get();
    }

    /**
     * <h3>loadedIsTouching</h3>
     * 
     * This method returns the loaded sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean loadedIsTouching() {
        return !loadedSensor.get();
    }

    /**
     * <h3>stagedIsTouching</h3>
     * 
     * This method returns the staged sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean stagedIsTouching() {
        return !stagedSensor.get();
    }

    /**
     * <h3>ejectionIsTouching</h3>
     * 
     * This method returns the ejection sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean ejectionIsTouching() {
        return !ejectionSensor.get();
    }

} // End of class IndexerSensorUtility
