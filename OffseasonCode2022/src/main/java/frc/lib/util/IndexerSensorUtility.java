package frc.lib.util;

import edu.wpi.first.wpilibj.DigitalInput;


public class IndexerSensorUtility {
    
    //-------- CONSTANTS --------\\

    private static final int INTAKE_ID = 1;
    private static final int LOADED_ID = 2;
    private static final int STAGED_ID = 3;
    private static final int EJECTION_ID = 4;
    
    //-------- VARIABLES --------\\
    
    private final DigitalInput intakeSensor;
    private final DigitalInput loadedSensor;
    private final DigitalInput stagedSensor;
    private final DigitalInput ejectionSensor;

    //-------- CONSTRUCTOR --------\\
    /**
     * <h3>EndgameSensorUtility</h3>
     * 
     * Creates and returns sensor values the endgame sensors
     */
    private IndexerSensorUtility() {
        intakeSensor = new DigitalInput(INTAKE_ID);
        loadedSensor = new DigitalInput(LOADED_ID);
        stagedSensor = new DigitalInput(STAGED_ID);
        ejectionSensor = new DigitalInput(EJECTION_ID);
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
