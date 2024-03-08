package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.lang.invoke.ConstantCallSite;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax leftFlywheelMotor = new CANSparkMax(Constants.Shooter.leftLeaderFlywheelMotor,
            MotorType.kBrushless);
    private CANSparkMax rightFlywheelMotor = new CANSparkMax(Constants.Shooter.rightLeaderFlywheelMotor,
            MotorType.kBrushless);

    private boolean PIDEnabled = false;

    private PIDController leftPID = new PIDController(Constants.Shooter.shooterkP, Constants.Shooter.shooterkI,
            Constants.Shooter.shooterkD);
    private PIDController rightPID = new PIDController(Constants.Shooter.shooterkP, Constants.Shooter.shooterkI,
            Constants.Shooter.shooterkD);
    private SimpleMotorFeedforward FF = new SimpleMotorFeedforward(Constants.Shooter.shooterks,
            Constants.Shooter.shooterkv);

    public SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism((Measure<Voltage> voltage) -> setFlywheelVoltage(voltage.in(Units.Volts), 0),
                    log ->
                    // Record a frame for the shooter motor.
                    log.motor("Flywheel")
                            .voltage(
                                    Units.Volts.of(
                                            leftFlywheelMotor.getAppliedOutput() * leftFlywheelMotor.getBusVoltage()))
                            .angularPosition(Units.Rotations.of(leftFlywheelMotor.getEncoder().getPosition()))
                            .angularVelocity(Units.RotationsPerSecond.of(getLeftFlywheelSpeed())),
                    this));

    public ShooterSubsystem() {
        leftFlywheelMotor.setInverted(Constants.Shooter.kleftMotorInverted);
        rightFlywheelMotor.setInverted(Constants.Shooter.krightMotorInverted);

    }

    public void setSpeed(double leftrpm, double rightrpm) {
        leftPID.setSetpoint(leftrpm); // Set the setpoint of the PID controller
        rightPID.setSetpoint(rightrpm);
    }

    public void setSpeed(double rpm) {
        setSpeed(rpm, rpm);
    }

    public void setFlywheelVoltage(double leftVoltage, double rightVoltage) {
        leftFlywheelMotor.setVoltage(leftVoltage); // Set the voltage of the motor
        rightFlywheelMotor.setVoltage(rightVoltage);
        SmartDashboard.putNumber("Left Flywheel", leftVoltage);
        SmartDashboard.putNumber("Right Flywheel", rightVoltage);
    }

    public void setIdleSpeed() {
        leftFlywheelMotor.set(1000);
        rightFlywheelMotor.set(1000);
    }

    public double getLeftFlywheelSpeed() {
        return leftFlywheelMotor.getEncoder().getVelocity() * Constants.Shooter.CONVERSION_FACTOR; // Get th // motor
    }

    public double getRightFlywheelSpeed() {
        return rightFlywheelMotor.getEncoder().getVelocity() * Constants.Shooter.CONVERSION_FACTOR; // Get the // motor
    }

    public void periodic() {
        SmartDashboard.putNumber("Left PID", leftPID.getSetpoint());
        SmartDashboard.putNumber("Right PID", rightPID.getSetpoint());
        SmartDashboard.putNumber("Left Current", getLeftFlywheelSpeed());
        SmartDashboard.putNumber("Right Current", getRightFlywheelSpeed());

        double leftVoltage = leftPID.calculate(getLeftFlywheelSpeed()) + FF.calculate(leftPID.getSetpoint());
        double rightVoltage = rightPID.calculate(getRightFlywheelSpeed()) + FF.calculate(rightPID.getSetpoint());

        if (PIDEnabled) {
            setFlywheelVoltage(leftVoltage, rightVoltage);
        }

    }

    public boolean isBusy() {
        return !(leftPID.atSetpoint() && rightPID.atSetpoint());

    }

    public void enable() {
        PIDEnabled = true;
    }

    public void disable() {
        PIDEnabled = false;
        setFlywheelVoltage(0, 0);
    }

    public boolean getEnabled(){
        return PIDEnabled;
    }

    public Command sysIdQuasistaticc(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
