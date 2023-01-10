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
    
    private final SparkMaxWrapper m_stagedIndexer;
    private final SparkMaxWrapper m_ejectionIndexer;

    // ----- CONSTRUCTOR -----\\
    /**
     * <h3>IndexerSubsystem</h3>
     * 
     * Manages the indexer motors.
     * 
     * @param stagedID - Can ID of the staged motor
     * @param ejectionID - Can ID of the ejection motor
     */
    public IndexerSubsystem(int stagedID, int ejectionID) {

        m_stagedIndexer = new SparkMaxWrapper(stagedID, MotorType.kBrushless);
        m_ejectionIndexer = new SparkMaxWrapper(ejectionID, MotorType.kBrushless);

    }

    // ----- METHODS -----\\

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
     * qu
     * Stops all two indexer motors.
     */
    public void stopMotors() {
        // Should stop all two motors
        m_stagedIndexer.stopMotor();
        m_ejectionIndexer.stopMotor();
        // Need to set voltages to zero because stop motor doesn't actually stop them?
    }
}
