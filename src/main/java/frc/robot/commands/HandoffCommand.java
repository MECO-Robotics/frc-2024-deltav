package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HandoffCommand extends Command {
    private final IndexingSubsystem indexer;
    private final IntakeSubsystem intake;

    private boolean finished = false;
    private boolean reversing = false;

    public HandoffCommand(IndexingSubsystem indexerPassedIn, IntakeSubsystem intakePassedIn) {
        indexer = indexerPassedIn;
        intake = intakePassedIn;
        addRequirements(indexer, intake);
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
        } else {
            indexer.setIndexingVoltage(6);
            intake.setIntakeVoltage(12);
            SmartDashboard.putBoolean("Is Handoff Command Intaking?", true);
        }
    }

    public void end(boolean isInterrupted) {
        indexer.setIndexingVoltage(0);
        intake.setIntakeVoltage(0);
    }

    public boolean isFinished() {
        return finished;
    }

}
