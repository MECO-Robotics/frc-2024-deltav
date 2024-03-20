package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LedDefaultCommand extends Command {
    private final LEDSubsystem led;
    private int increment = 0;

    public LedDefaultCommand(LEDSubsystem ledPassedIn) {
        led = ledPassedIn;
        addRequirements(led);
    }

    public void execute() {
        Color color = Color.kBlack;
        if (DriverStation.isTeleopEnabled() && DriverStation.getAlliance().isPresent()) { // Teleop Enabled - Solid alliance color

            Alliance ally = DriverStation.getAlliance().get();

            if (ally == Alliance.Red) {
                color = new Color(16, 0, 0);
            } else if (ally == Alliance.Blue) { // should only have pipelines 0 & 1
                color = new Color(0, 0, 16);
            }
            led.setAll(color);

        } else if (DriverStation.isAutonomousEnabled() && DriverStation.getAlliance().isPresent()) { // Auto Enabled - Blink alliance color

            led.setAll(Color.kBlack);

            Alliance ally = DriverStation.getAlliance().get();

            if (ally == Alliance.Red) {
                color = new Color(16, 0, 0);
            } else if (ally == Alliance.Blue) { // should only have pipelines 0 & 1
                color = new Color(0, 0, 16);
            }

            increment++;

            if (increment < 12) {

                led.setAll(color);

            } else if (increment >= 12 && increment <= 25) {

                led.setAll(Color.kBlack);

            } else {
                increment = 0;
            }

        } else { // Disabled or anything else - Rainbow pattern
            led.rainbow();
        }

    }

    public boolean isFinished() {
        return false;
    }
}
