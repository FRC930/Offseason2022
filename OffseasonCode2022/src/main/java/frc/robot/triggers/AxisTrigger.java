package frc.robot.triggers;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AxisTrigger extends Trigger {
    private final double TRIGGER_PRESSED_THRESHOLD = 0.4;

    private XboxController controller;
    private int axisNumber;

    public AxisTrigger(XboxController controller, int axisNum) {
        requireNonNullParam(controller, "Controller", "AxisTrigger Constructor");

        this.controller = controller;
        axisNumber = axisNum;
    }

    @Override
    public boolean get() {
        return (this.controller.getRawAxis(axisNumber) >= TRIGGER_PRESSED_THRESHOLD);
    }
}
