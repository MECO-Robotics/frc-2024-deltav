package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class StartIntakingCommand extends Command {
    
    private final ArmSubsystem shooter ;
    private final IntakeSubsystem intake  ;

    public StartIntakingCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem){
        shooter = armSubsystem;
        intake = intakeSubsystem;
    }
    public void execute() {
       
        
        if(shooter.isNoteAquired()){
        intake.stopIntaking(isScheduled());
        }
        else{
        intake.startIntaking(true);
        }
        
    }
    public boolean isFinished() {
        return true;
    }







    
}
