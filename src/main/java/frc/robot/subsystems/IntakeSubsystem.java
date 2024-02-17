package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Controls the Intake motors and sensors, and contains all intake-related commands
public class IntakeSubsystem extends SubsystemBase {

  

    private CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.intakeMotorCANID, MotorType.kBrushless);

    

    public IntakeSubsystem() {
        


        DigitalInput intakeBeamBreak = new DigitalInput(Constants.Intake.intakeBeamBreakDIOPort);

        // intakeMotorLeft.follow(intakemotorRight, true)
        // should only need one motor for intake but have capabilities for two
    }

    // intake start command
    public void startIntaking(boolean shooterEmpty) {
        //start wheels at kIntakingSpeed
        
    }

    // handoff to shooter command
    public void handoffNote() {
        //set wheels to kHandoffSpeed (slower than intaking?)

    }

    // emergency outtake command
    public void ejectIntake() {
        //set wheels to -(kIntakingSpeed)
    }

}
