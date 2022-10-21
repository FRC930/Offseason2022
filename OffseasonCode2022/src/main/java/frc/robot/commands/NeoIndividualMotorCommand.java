package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoMotorSubsystem;

public class NeoIndividualMotorCommand extends CommandBase {
    
    private final double MOTOR_SPEED;

    private final NeoMotorSubsystem neoMotorSubsystem;

    public NeoIndividualMotorCommand(NeoMotorSubsystem neoMotor, boolean isBackwards){
        if(isBackwards){
            MOTOR_SPEED = 0.25;
        }
        else{
            MOTOR_SPEED = -0.25;
        }

        neoMotorSubsystem = neoMotor;
    }

    @Override
    public void initialize() {
        neoMotorSubsystem.setMotorSpeed(MOTOR_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        neoMotorSubsystem.stopMotor();
    }
}
