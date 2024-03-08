package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class NoAutomationIntakieCommand extends Command {
    
   
    private final IntakeSubsystem intake;
    private final DoubleSupplier intakeSpeed;

    public NoAutomationIntakieCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier intakeSpeed){
        intake = intakeSubsystem;
        this.intakeSpeed = intakeSpeed;
        addRequirements(intakeSubsystem);
    }
    public void execute() {
        intake.setIntakeVoltage(intakeSpeed.getAsDouble());

    }
    public boolean isFinished() {
        
        return intakeSpeed == null;
    }







    
}
