package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
/* 
public class ManualArmControlCommand extends Command{
    
    private final ArmSubsystem arm;
    private final DoubleSupplier input;

    public ManualArmControlCommand(ArmSubsystem armSubsystem, DoubleSupplier controllerInput){
        arm = armSubsystem;
        input = controllerInput;

    }
    public void execute(){
        double returnedValue = input.getAsDouble();
        arm.manualArmControl(returnedValue);
    }
    public boolean isFinished(){
        return false;
    }
}
*/
