package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//This command never finishees unless end if called

public class ShooterCommand extends Command{

    private final ShooterSubsystem shooter;

    public ShooterCommand(ShooterSubsystem shooterSubsystem){
        shooter = shooterSubsystem;
    }
    public void execute(){
        shooter.setSpeed(Constants.Shooter.Presets.kLeftSpeaker, Constants.Shooter.Presets.kRightSpeaker);
    }
    public boolean isFinished(){
        return !shooter.isBusy();
    }

}
