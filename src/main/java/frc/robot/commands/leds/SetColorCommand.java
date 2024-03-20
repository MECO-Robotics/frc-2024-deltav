package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class SetColorCommand extends Command {

    private final LEDSubsystem led;
    private final Color color;

    public SetColorCommand(LEDSubsystem ledPassedIn, Color colorPassedIn) {
        led = ledPassedIn;
        color = colorPassedIn;
        addRequirements(led);
    }

    public void initialize() {
        led.setAll(color);
    }

    public boolean isFinished() {
        return true;
    }
    
}
