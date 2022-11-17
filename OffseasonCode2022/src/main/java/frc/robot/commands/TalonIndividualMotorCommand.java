package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TalonMotorSubsystem;

public class TalonIndividualMotorCommand extends CommandBase {
    
    private final double MOTOR_SPEED;

    private final TalonMotorSubsystem talonMotorSubsystem;

    public TalonIndividualMotorCommand(TalonMotorSubsystem talonMotor, boolean isBackwards){
        if(isBackwards){
            MOTOR_SPEED = 0.25;
        }
        else{
            MOTOR_SPEED = -0.25;
        }

        talonMotorSubsystem = talonMotor;
    }

    @Override
    public void initialize() {
        talonMotorSubsystem.setMotorSpeed(MOTOR_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        talonMotorSubsystem.stopMotor();
    }
}
