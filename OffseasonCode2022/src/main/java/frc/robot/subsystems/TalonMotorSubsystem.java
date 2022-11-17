package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class TalonMotorSubsystem {

    WPI_TalonFX talonFX;

    public TalonMotorSubsystem(int canID){
        talonFX = new WPI_TalonFX(canID);
    }

    public void setMotorSpeed(double speed){
        talonFX.set(speed);
    }

    public void stopMotor(){
        talonFX.set(0.0);
    }
}
