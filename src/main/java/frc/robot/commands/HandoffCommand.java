package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
/* 
public class HandoffCommand extends Command{
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private boolean isDone = false;    

    public HandoffCommand(ShooterSubsystem shooterPassedIn, IntakeSubsystem intakePassedIn){
        shooter = shooterPassedIn;
        intake = intakePassedIn;
    }
    public void execute(){
        boolean shooterIsDone = shooter.pullNoteIn();
        boolean intakeIsDone = intake.handoffNote();
        isDone = shooterIsDone && intakeIsDone;

    }
    public boolean isFinished(){
        return isDone;
    }



}
*/
