// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ControllerSubsystem extends SubsystemBase {

    private final XboxController pilotController = new XboxController(0);
    private final XboxController copilotController = new XboxController(1);

    // Only use if joystick control is going to be used.
    private final Joystick joystick = null; // new Joystick(0);

    public ControllerSubsystem() {
    }

    public XboxController getPilotController() {
        return pilotController;
    }

    public XboxController getCopilotController() {
        return copilotController;
    }

    public Joystick getJoystick() {
        return joystick;
    }

    @Override
    public void periodic() {
    }

    /**
     * Return value unless it is within deadzone of zero, then just return zero.
     */
    public static double deadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone)
            return 0;
        return value;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}
