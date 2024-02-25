package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

//This command never finishees unless end if called

public class ShooterCommand extends Command{

    private final ArmSubsystem arm;

    public ShooterCommand(ArmSubsystem armSubsystem){
        arm = armSubsystem;
    }
    public void execute(){
        arm.shootSpeaker();
    }
    public void end(boolean interrupted) {
        arm.idleFlywheels();
    }
    public boolean isFinished(){
        return false;
    }

}
