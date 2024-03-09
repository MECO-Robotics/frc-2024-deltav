package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

//This command never finishees unless end if called

public class ShooterCommand extends Command {

    private final ShooterSubsystem shooter;
    private final double leftRPM, rightRPM;
    private boolean isFinished = false;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double leftRPM, double rightRPM) {
        shooter = shooterSubsystem;
        this.leftRPM = leftRPM;
        this.rightRPM = rightRPM;
    }

    public void initialize() {
        shooter.enable();
        shooter.setSpeed(leftRPM, rightRPM);
    }

    public void execute() {
        if (shooter.getEnabled() && !shooter.isBusy()) {
            isFinished = true;
        }
    }

    public boolean isFinished() {
        return isFinished;
    }

}
