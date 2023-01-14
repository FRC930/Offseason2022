//imports
package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.utilities.SparkMaxWrapper;
//This command controls motors for the intake
public class NeoMotorSubsystem {
    
    SparkMaxWrapper neoMotor;

    public NeoMotorSubsystem(int canID){
        neoMotor = new SparkMaxWrapper(canID, MotorType.kBrushless);
    }

    public void setMotorSpeed(double speed){
        neoMotor.set(speed);
    }

    public void stopMotor(){
        neoMotor.set(0.0);
    }
}
