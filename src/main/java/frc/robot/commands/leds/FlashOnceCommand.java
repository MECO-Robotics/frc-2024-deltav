package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LED;
import frc.robot.subsystems.LEDSubsystem;

public class FlashOnceCommand extends SequentialCommandGroup {

    public FlashOnceCommand(LEDSubsystem led, Color color) {
        addRequirements(led); // ensures that we don't run the default command while we're running the wait command.

        addCommands(
            new SetColorCommand(led, Color.kBlack),
            new WaitCommand(0.1),
            new SetColorCommand(led, color),
            new WaitCommand(0.5),
            new SetColorCommand(led, Color.kBlack)
        );

    }

}
