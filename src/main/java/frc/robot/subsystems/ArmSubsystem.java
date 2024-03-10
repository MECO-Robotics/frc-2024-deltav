package frc.robot.subsystems;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

// Controls the Arm and Shooter motors and sensors, and contains all Arm-, Shooter-, and Climber-related commands
public class ArmSubsystem extends TrapezoidProfileSubsystem {
    private CANSparkMax rightArmMotorOne = new CANSparkMax(Constants.Arm.rightMotorOneID, MotorType.kBrushless);
    private CANSparkMax rightArmMotorTwo = new CANSparkMax(Constants.Arm.rightMotorTwoID, MotorType.kBrushless);

    private CANSparkMax leftArmMotorOne = new CANSparkMax(Constants.Arm.leftMotorOneID, MotorType.kBrushless);
    private CANSparkMax leftArmMotorTwo = new CANSparkMax(Constants.Arm.leftMotorTwoID, MotorType.kBrushless);
    
    private DutyCycleEncoder armEncoder = new DutyCycleEncoder(Constants.Arm.armEncoderPortS);
    // private Encoder armIncrementalEncoder = new Encoder(Constants.Arm.armEncoderPortA, Constants.Arm.armEncoderPortB);
        
    private double initialArmPosition;

    private PIDController PID = new PIDController(Constants.Arm.armkP, Constants.Arm.armkI, Constants.Arm.armkD);
    private ArmFeedforward FF = new ArmFeedforward(Constants.Arm.armks, Constants.Arm.armkg, Constants.Arm.armkv);

    public SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism((Measure<Voltage> voltage) -> setVoltage(voltage.in(Units.Volts)), log ->
            // Record a frame for the shooter motor.
            log.motor("arm")
                    .voltage(
                            Units.Volts.of(leftArmMotorOne.getAppliedOutput() * leftArmMotorOne.getBusVoltage()))
                    .angularPosition(Units.Rotations.of(getPosition()))
                    .angularVelocity(Units.RotationsPerSecond.of(getVelocity())), this));

    // Constructor
    public ArmSubsystem() {
        super(Constants.Arm.kArmMotionConstraint);
        leftArmMotorTwo.follow(leftArmMotorOne, false);
        rightArmMotorOne.follow(leftArmMotorOne, true);
        rightArmMotorTwo.follow(leftArmMotorOne, true);
        leftArmMotorOne.getEncoder().setVelocityConversionFactor(Constants.Arm.ajustedArmGearRatio);
        leftArmMotorOne.getEncoder().setPositionConversionFactor(Constants.Arm.ajustedArmGearRatio);

        armEncoder.setPositionOffset(Constants.Arm.horizontalArmOffset);
        // armIncrementalEncoder.setDistancePerPulse(1.0/Constants.Arm.armTicksPerRevolution);
        // armIncrementalEncoder.setReverseDirection(true);
        initialArmPosition = armEncoder.get();
    }

    public void setVoltage(double voltage) {
        leftArmMotorOne.setVoltage(voltage);
    }

    public void periodic(){
        // SmartDashboard.putNumber("Arm Quadrature Encoder", armIncrementalEncoder.getDistance());
        SmartDashboard.putNumber("Duty Cycle Encoder", armEncoder.get());
    }

    private double getPosition() {
        return armEncoder.get();
    }

    public double getVelocity() {
        return armEncoder.get();
    }

    public void useState(TrapezoidProfile.State state) {
        SmartDashboard.putNumber("state velocity", state.velocity);
        SmartDashboard.putNumber("State position", state.position);
        setVoltage(PID.calculate(getVelocity(), state.velocity) + FF.calculate(state.position, state.velocity));
    }

    public boolean isBusy() {
        return !PID.atSetpoint();
    }

    public Command sysIdQuasistaticc(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

}
