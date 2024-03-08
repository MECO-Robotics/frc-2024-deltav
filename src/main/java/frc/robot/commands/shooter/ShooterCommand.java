package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

//This command never finishees unless end if called

public class ShooterCommand extends Command {

    private final ShooterSubsystem shooter;
    private final double leftRPM, rightRPM;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double leftRPM, double rightRPM) {
        shooter = shooterSubsystem;
        this.leftRPM = leftRPM;
        this.rightRPM = rightRPM;
    }

    public void initialize() {
        shooter.enable();
        shooter.setSpeed(leftRPM, rightRPM);
        SmartDashboard.putBoolean("Flywheel enabled", true);
    }

    public boolean isFinished() {
        SmartDashboard.putBoolean("Flywheel enabled", false);

        return !shooter.isBusy();
    }

}
