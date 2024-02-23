package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class StartIntakingCommand extends Command {
    
    private final ArmSubsystem shooter ;
    private final IntakeSubsystem intake  ;

    public StartIntakingCommand(ArmSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem){
        shooter = shooterSubsystem;
        intake = intakeSubsystem;
    }
    public void execute() {
        shooter.
        intake.startIntaking(true);
    }
    public boolean isFinished() {
        return true;
    }







    
}
