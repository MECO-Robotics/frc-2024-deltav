package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexingCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private final DoubleSupplier indexingSpeed;
    public IndexingCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier indexingSpeed){
        this.shooterSubsystem = shooterSubsystem;
        this.indexingSpeed = indexingSpeed;
        addRequirements(shooterSubsystem);
    }
    public void execute(){
        shooterSubsystem.setIndexingVoltage(indexingSpeed.getAsDouble());
    
    }
    public boolean isFinished(){
        return false;
    }
}
