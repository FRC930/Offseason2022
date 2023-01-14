/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkMaxWrapper;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//-------- SUBSYSTEM CLASS --------\\

/**
 * <h3>LoadedMotorSubsystem</h3>
 * Subsystem class to manage the loaded motor.
 */
public class LoadedMotorSubsystem extends SubsystemBase {

    // -------- DECLARATIONS --------\\
    // motor controllers for the loaded motor
    private final SparkMaxWrapper Loaded;

    // -------- CONSTRUCTOR --------\\
    /**
     * <h3>LoadedMotorSubsystem</h3>
     * Creates a subsystem class to manage the loaded motor.
     * 
     * @param LoadedID The ID of the loaded motor
     * 
     */
    public LoadedMotorSubsystem(int LoadedID) {

        // Motor declaration
        Loaded = new SparkMaxWrapper(LoadedID, MotorType.kBrushless);

        // Reset motors
        Loaded.restoreFactoryDefaults();

        // Sets motors to coast so that they can move freely when neutral
        // TODO determine how on sparkmaxLoaded.setNeutralMode(NeutralMode.Coast);
    }

    // -------- METHODS --------\\
    /**
     * <h3>setSpeed</h3>
     * Sets the loaded motor speed.
     *
     * @param speed speed of the loaded motor in percent output
     */
    public void setSpeed(double speed) {
        Loaded.set(speed);
    }
    
} // end of class LoadedMotorSubsystem