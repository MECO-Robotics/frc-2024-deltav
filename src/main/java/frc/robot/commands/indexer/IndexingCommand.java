package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexingSubsystem;

public class IndexingCommand extends Command{
    private final IndexingSubsystem indexingSubsystem;
    private final DoubleSupplier indexingSpeed;
    public IndexingCommand(IndexingSubsystem indexingSubsystem, DoubleSupplier indexingSpeed){
        this.indexingSubsystem = indexingSubsystem;
        this.indexingSpeed = indexingSpeed;
        addRequirements(indexingSubsystem);
    }
    public void execute(){
        indexingSubsystem.setIndexingVoltage(indexingSpeed.getAsDouble());
    
    }
    public boolean isFinished(){
        return false;
    }
}
