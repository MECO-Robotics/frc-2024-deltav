package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HandoffCommand extends Command {
    private final IndexingSubsystem indexer;
    private final IntakeSubsystem intake;
    private final XboxController pilot;
    private final XboxController copilot;

    private boolean finished = false;
    private boolean reversing = false;

    public HandoffCommand(IndexingSubsystem indexerPassedIn, IntakeSubsystem intakePassedIn, XboxController pilot,
            XboxController copilot) {
        indexer = indexerPassedIn;
        intake = intakePassedIn;
        this.pilot = pilot;
        this.copilot = copilot;
        addRequirements(indexer, intake);
    }

    public HandoffCommand(IndexingSubsystem indexerPassedIn, IntakeSubsystem intakePassedIn) {
        this(indexerPassedIn, intakePassedIn, null, null);
    }

    public void initialize() {
        finished = false;
        reversing = false;
    }

    public void execute() {
        if (indexer.isNoteAquired()) {
            indexer.setIndexingVoltage(-6);
            intake.setIntakeVoltage(0);
            reversing = true;
            SmartDashboard.putBoolean("Is Handoff Command Reversing?", true);
        } else if (reversing) {
            finished = true;
            SmartDashboard.putBoolean("Is Handoff Command Reversing?", false);
            SmartDashboard.putBoolean("Is Handoff Command Intaking?", false);
            if (pilot != null) {
                pilot.setRumble(RumbleType.kBothRumble, 1);
                copilot.setRumble(RumbleType.kBothRumble, 1);
            }
        } else {
            indexer.setIndexingVoltage(6);
            intake.setIntakeVoltage(12);
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
    }

    public boolean isFinished() {
        return finished;
    }

}
