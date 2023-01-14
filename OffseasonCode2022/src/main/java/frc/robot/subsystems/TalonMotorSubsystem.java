//imports//
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class TalonMotorSubsystem {

    WPI_TalonFX talonFX;
//sets motor id's in can
    public TalonMotorSubsystem(int canID){
        talonFX = new WPI_TalonFX(canID);
    }
//changes speed of motor
    public void setMotorSpeed(double speed){
        talonFX.set(speed);
    }
//resets motor
    public void stopMotor(){
        talonFX.set(0.0);
    }
}
//end of class