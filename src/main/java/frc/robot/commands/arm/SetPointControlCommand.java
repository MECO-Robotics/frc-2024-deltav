package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetPointControlCommand extends Command {

    private final ArmSubsystem arm;
    private final double input;

    public SetPointControlCommand(ArmSubsystem armSubsystem, double armPosition) {
        arm = armSubsystem;
        input = armPosition;
        addRequirements(armSubsystem);
        SmartDashboard.putNumber("Arm Setpoint", armPosition);
        

    }

    public void execute() {
        //arm.enable();
        arm.setPosition(input);
    }

    public boolean isFinished() {
        return false;// !arm.isBusy();
    }
}
