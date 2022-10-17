//----- IMPORTS -----\\
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkMaxWrapper;

//----- CLASS -----\\
/**
 * <h3>IndexerSubsystem</h3>
 * 
 * Manages the indexer motors.
 */
public class IndexerSubsystem extends SubsystemBase {

    //-------- VARIABLES --------\\
    
    private final SparkMaxWrapper m_loadedIndexer;
    private final SparkMaxWrapper m_stagedIndexer;
    private final SparkMaxWrapper m_ejectionIndexer;

    // ----- CONSTRUCTOR -----\\
    /**
     * <h3>IndexerSubsystem</h3>
     * 
     * Manages the indexer motors.
     * 
     * @param stagedID - Can ID of the staged motor
     * @param loadedID - Can ID of the loaded motor
     * @param ejectionID - Can ID of the ejection motor
     */
    public IndexerSubsystem(int loadedID, int stagedID, int ejectionID) {

        m_loadedIndexer = new SparkMaxWrapper(loadedID, MotorType.kBrushless);
        m_stagedIndexer = new SparkMaxWrapper(stagedID, MotorType.kBrushless);
        m_ejectionIndexer = new SparkMaxWrapper(ejectionID, MotorType.kBrushless);

    }

    // ----- METHODS -----\\

    /**
     * <h3>setLoadedMotorSpeed</h3>
     * 
     * Sets the loaded indexer motor speed.
     * 
     * @param speed the speed to set the motor
     */
    public void setLoadedMotorSpeed(double speed) {
        m_loadedIndexer.set(speed);
    }

    /**
     * <h3>setStagedMotorSpeed</h3>
     * 
     * Sets the staged indexer motor speed.
     * 
     * @param speed the speed to set the motor
     */
    public void setStagedMotorSpeed(double speed) {
        m_stagedIndexer.set(speed);
    }

    /**
     * <h3>setEjectionMotorSpeed</h3>
     * 
     * Sets the staged ejection motor speed
     * 
     * @param speed the speed to set the motor
     */
    public void setEjectionMotorSpeed(double speed) {
        m_ejectionIndexer.set(speed);
    }

    /**
     * <h3>stopMotors</h3>
     * 
     * Stops all three indexer motors.
     */
    public void stopMotors() {
        // Should stop all three motors
        m_stagedIndexer.stopMotor();
        m_loadedIndexer.stopMotor();
        m_ejectionIndexer.stopMotor();
        // Need to set voltages to zero because stop motor doesn't actually stop them?
    }
}
