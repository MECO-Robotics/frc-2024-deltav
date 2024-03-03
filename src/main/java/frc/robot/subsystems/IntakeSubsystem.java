package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Controls the Intake motors and sensors, and contains all intake-related commands
public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.intakeMotorCANID, MotorType.kBrushless);

    DigitalInput intakeBeambreak = new DigitalInput(Constants.Intake.kBeamBreakSensorPort);

    public IntakeSubsystem() {

        // DigitalInput intakeBeamBreak = new
        // DigitalInput(Constants.Intake.intakeBeamBreakDIOPort);

        // intakeMotorLeft.follow(intakemotorRight, true)
        // should only need one motor for intake but have capabilities for two
    }

    // intake start command
    public void startIntaking(boolean shooterEmpty) {
        // start wheels at kIntakingSpeed
        intakeMotor.setVoltage(6);

    }

    public void setIntakeVoltage(double voltage) {
        // start wheels at kIntakingSpeed
        intakeMotor.setVoltage(voltage);
    }

    public void stopIntaking(boolean shooterEmpty) {
        intakeMotor.setVoltage(0);
    }

    // handoff to shooter command
    public boolean handoffNote() {
        // set wheels to kHandoffSpeed (slower than intaking?)

        if (intakeBeambreak.get()) {
            // NOTE is in
            intakeMotor.set(.8);
            return false;
        } else {
            // NOTE is In
            intakeMotor.set(0);
            return true;
        }

    }

    // emergency outtake command
    public void ejectIntake() {
        // set wheels to -(kIntakingSpeed)
        intakeMotor.setVoltage(-6);
    }

}
