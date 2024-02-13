package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO Add a class comment. What does this class do?
public class IntakeSubsystem extends SubsystemBase {

    // TODO Declare the motors used on the intake. Something like:
    // private CANSparkMax someMotor = new CANSparkMax(Constants.kSomeCanIDConstant,
    // MotorType.kBrushless);

    // TODO Declare a constructor like this:
    // public IntakeSubsystem() {
    // // Do initialization stuff for the class
    //
    // // TODO In the constructor link up any motors that should follow another one,
    //              and specify true as the second parameter if it should be inverted
    // // someMotor.follow(someOppositeMotor, true);
    //
    // }

    // intake initial command
    public void startIntaking(boolean shooterEmpty, boolean armDown) {
    }

    // handoff to shooter command
    public void handoffNote() {

    }

    // emergency outtake command
    public void ejectIntake(boolean ejectOn) {

    }

}
