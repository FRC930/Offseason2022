package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ShuffleboardUtility;

public class ShuffleboardSubsystem extends SubsystemBase {
    private ShuffleboardUtility mShuffleBoard ;

    public ShuffleboardSubsystem() {
        mShuffleBoard = ShuffleboardUtility.getInstance() ;
    }

    @Override
    public void periodic() {
        mShuffleBoard.update() ;
    }

    @Override
    public void simulationPeriodic() {
        mShuffleBoard.update() ;
    }
}
