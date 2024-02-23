package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntakingCommand extends Command {
    
    private final ArmSubsystem shooter ;
    private final IntakeSubsystem intake  ;

    public StopIntakingCommand(ArmSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem){
        shooter = shooterSubsystem;
        intake = intakeSubsystem;
    }
    public void execute() {
        // TODO stop intaking
    }
    public boolean isFinished() {
        // TODO determine if this should return true or false
        return true;
    }







    
}
