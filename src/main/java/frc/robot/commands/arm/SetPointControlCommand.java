package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetPointControlCommand extends Command {

    private final ArmSubsystem arm;
    private final DoubleSupplier input;
    private final boolean auto;

    public SetPointControlCommand(ArmSubsystem armSubsystem, double armPosition) {
        arm = armSubsystem;
        input = () -> armPosition;
        auto = true;
        addRequirements(armSubsystem);
    }

    public SetPointControlCommand(ArmSubsystem armSubsystem, DoubleSupplier armPosition) {
        arm = armSubsystem;
        input = armPosition;
        auto = false;
        addRequirements(armSubsystem);
    }

    public void execute() {
        SmartDashboard.putNumber("Arm Setpoint", input.getAsDouble());
        arm.setPosition(input.getAsDouble());
        arm.enable();
    }
    public boolean isFinished() {
        return auto && !arm.isBusy();
    }
}
