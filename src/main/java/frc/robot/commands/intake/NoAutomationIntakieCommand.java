package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class NoAutomationIntakieCommand extends Command {
    
   
    private final IntakeSubsystem intake  ;

    public NoAutomationIntakieCommand( IntakeSubsystem intakeSubsystem){
        intake = intakeSubsystem;
    }
    public void execute() {
        intake.startIntaking(true);
    }
    public boolean isFinished() {
        
        return true;
    }







    
}
