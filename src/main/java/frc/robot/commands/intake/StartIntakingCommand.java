package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartIntakingCommand extends Command {
    
    private final ShooterSubsystem shooter ;
    private final IntakeSubsystem intake  ;

    public StartIntakingCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem){
        shooter = shooterSubsystem;
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
