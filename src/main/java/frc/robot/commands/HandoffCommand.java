package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
        } else if (reversing) {
            finished = true;
        } else {
            indexer.setIndexingVoltage(6);
            intake.setIntakeVoltage(12);
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
