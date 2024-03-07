package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private boolean PIDEnabled;

    private PIDController leftPID = new PIDController(Constants.Shooter.shooterkP, Constants.Shooter.shooterkI, Constants.Shooter.shooterkD);
    private PIDController rightPID = new PIDController(Constants.Shooter.shooterkP, Constants.Shooter.shooterkI, Constants.Shooter.shooterkD);
    private SimpleMotorFeedforward FF = new SimpleMotorFeedforward(Constants.Shooter.shooterks,Constants.Shooter.shooterkv);

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

    public void setIdleSpeed(){
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
        if(PIDEnabled){
            setFlywheelVoltage(leftPID.calculate(getLeftFlywheelSpeed()) + FF.calculate(leftPID.getSetpoint()),
                rightPID.calculate(getRightFlywheelSpeed())+FF.calculate(rightPID.getSetpoint()));
        }

    }

    public boolean isBusy() {
        return !(leftPID.atSetpoint() && rightPID.atSetpoint());

    }

    public boolean isNoteAquired() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isNoteAquired'");
    }

    public void setIndexingVoltage(double asDouble) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setIndexingVoltage'");
    }
    public void enable(){
        PIDEnabled = true;
    }
    public void disabled(){
        PIDEnabled = false;
        setFlywheelVoltage(0, 0);
    }
}
