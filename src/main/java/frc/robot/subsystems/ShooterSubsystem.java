package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;


import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    private CANSparkMax leftFlywheelMotor = new CANSparkMax(Constants.Shooter.leftLeaderFlywheelMotor,
            MotorType.kBrushless);
    private CANSparkMax rightFlywheelMotor = new CANSparkMax(Constants.Shooter.rightLeaderFlywheelMotor,
            MotorType.kBrushless);
    private CANSparkMax indexingMotor = new CANSparkMax(Constants.Shooter.indexingMotor, MotorType.kBrushless);

    private PIDController leftPID = new PIDController(Constants.Shooter.shooterkP, 0, 0);
    private PIDController rightPID = new PIDController(Constants.Shooter.shooterkP, 0, 0);
    private SimpleMotorFeedforward FF = new SimpleMotorFeedforward(Constants.Shooter.shooterks, Constants.Arm.armkg, Constants.Arm.armkv);
    
    DigitalInput indexerBeambreak = new DigitalInput(Constants.Shooter.shooterBeamBreakDIOPort);

    public ShooterSubsystem(){
        leftFlywheelMotor.setInverted(Constants.Shooter.kleftMotorInverted);
        rightFlywheelMotor.setInverted(Constants.Shooter.krightMotorInverted);
        indexingMotor.setInverted(Constants.Shooter.indexingMotorInverted);
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
    }

    public void setIndexingVoltage(double indexingVoltage){
        indexingMotor.setVoltage(indexingVoltage);
    }
    public double getLeftFlywheelSpeed() {
        return leftFlywheelMotor.getEncoder().getVelocity() * Constants.Shooter.CONVERSION_FACTOR; // Get th                                                                                            // motor
    }

    public double getRightFlywheelSpeed() {
        return rightFlywheelMotor.getEncoder().getVelocity() * Constants.Shooter.CONVERSION_FACTOR; // Get the                                                                                               // motor
    }

    public void periodic(){
        setFlywheelVoltage(leftPID.calculate(getLeftFlywheelSpeed()) + FF.calculate(leftPID.getSetpoint()), 
        FF.calculate(rightPID.getSetpoint()));
    }

    public boolean isBusy(){
        return !(leftPID.atSetpoint() && rightPID.atSetpoint());
        
    }
    
    
    public boolean isNoteAquired() {
        // return shooterBeamBreak.get();
        return false;
    }

   
    
}
