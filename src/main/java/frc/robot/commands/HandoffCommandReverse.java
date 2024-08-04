package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.leds.FlashOnceCommand;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.BlinkLimelightCommand;


public class HandoffCommandReverse extends Command {
    private final IndexingSubsystem indexer;
    private final IntakeSubsystem intake;
    private final LEDSubsystem led;
    private final XboxController pilot;
    private final XboxController copilot;

    private boolean finished = false;
    private boolean reversing = false;

    public 
    HandoffCommandReverse(IndexingSubsystem indexerPassedIn, IntakeSubsystem intakePassedIn, LEDSubsystem ledPassedIn, XboxController pilot,
            XboxController copilot) {
        indexer = indexerPassedIn;
        intake = intakePassedIn;
        led = ledPassedIn;
        this.pilot = pilot;
        this.copilot = copilot;
        addRequirements(indexer, intake, led);
    }

    public HandoffCommandReverse(IndexingSubsystem indexerPassedIn, IntakeSubsystem intakePassedIn, LEDSubsystem ledPassedIn) {
        this(indexerPassedIn, intakePassedIn, ledPassedIn, null, null);
    }

    public void initialize() {
        finished = false;
        reversing = false;
    }

    public void execute() {
        //signal
        if (pilot != null) {
                pilot.setRumble(RumbleType.kBothRumble, 1);
                copilot.setRumble(RumbleType.kBothRumble, 1);
            }

        led.chaserIndex(true);

        //actual handoff
        if (indexer.isNoteAquired()) {
            indexer.setIndexingVoltage(-6);
            intake.setIntakeVoltage(0);
            reversing = true;
            SmartDashboard.putBoolean("Is Handoff Command Reversing?", true);
        } else if (reversing) {
            finished = true;
            SmartDashboard.putBoolean("Is Handoff Command Reversing?", false);
            SmartDashboard.putBoolean("Is Handoff Command Intaking?", false);
            
        } else {
            indexer.setIndexingVoltage(6);
            intake.setIntakeVoltage(-12);
            SmartDashboard.putBoolean("Is Handoff Command Intaking?", true);
           
        }
    }

    public void end(boolean isInterrupted) {
        indexer.setIndexingVoltage(0);
        intake.setIntakeVoltage(0);

        if (pilot != null) {
                pilot.setRumble(RumbleType.kBothRumble, 0);
                copilot.setRumble(RumbleType.kBothRumble, 0);
        }
        led.chaserIndex(false);
        


        // if(!isInterrupted) {
        // CommandScheduler.getInstance().schedule(new FlashOnceCommand(led, Color.kGreen));
        // }
    }

    public boolean isFinished() {
        return finished;
    }

}
