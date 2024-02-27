package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HandoffCommand extends Command{
    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;
    private boolean isDone = false;    

    public HandoffCommand(ArmSubsystem armPassedIn, IntakeSubsystem intakePassedIn){
        arm = armPassedIn;
        intake = intakePassedIn;
    }
    public void execute(){
        boolean armIsDone = arm.pullNoteIn();
        boolean intakeIsDone = intake.handoffNote();
        isDone = armIsDone && intakeIsDone;

    }
    public boolean isFinished(){
        return isDone;
    }



}
