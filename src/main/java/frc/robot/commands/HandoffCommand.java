package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class HandoffCommand extends Command{
    private final IndexingSubsystem indexer;
    private final IntakeSubsystem intake;
      

    public HandoffCommand(IndexingSubsystem indexerPassedIn, IntakeSubsystem intakePassedIn){
        indexer = indexerPassedIn;
        intake = intakePassedIn;
        addRequirements(indexer, intake);
    }
    public void execute(){
        indexer.setIndexingVoltage(11);
        intake.setIntakeVoltage(11);
    }

    public void end(boolean isInterrupted){
        indexer.setIndexingVoltage(0);
        intake.setIntakeVoltage(0);
    }
    public boolean isFinished(){
        return indexer.isNoteAquired();
    }



}

