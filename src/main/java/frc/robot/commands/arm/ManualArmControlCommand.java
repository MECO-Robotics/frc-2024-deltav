package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmControlCommand extends Command{
    
    private final ArmSubsystem arm;
    private final DoubleSupplier input;

    public ManualArmControlCommand(ArmSubsystem armSubsystem, DoubleSupplier controllerInput){
        arm = armSubsystem;
        input = controllerInput;

        addRequirements(armSubsystem);
    }
    public void execute(){
        double returnedValue = input.getAsDouble();
        arm.set(returnedValue * 0.25);
        // arm.setVoltage(returnedValue * 12);
        arm.disable();
    }
    public boolean isFinished(){
        return false;
    }
}

