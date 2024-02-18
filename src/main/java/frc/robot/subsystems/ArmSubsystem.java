package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;



// Controls the Arm and Shooter motors and sensors, and contains all Arm-, Shooter-, and Climber-related commands
public class ArmSubsystem extends SubsystemBase {
    

   /* ------------------------------------------------------- *\
   |                       D E C L A R E                       |
   \* ------------------------------------------------------- */

    private CANSparkMax leftLeaderFlywheelMotor = new CANSparkMax(Constants.Shooter.leftLeaderFlywheelMotor, MotorType.kBrushless);
    private CANSparkMax rightFollowerFlywheelMotor = new CANSparkMax(Constants.Shooter.rightFollowerFlywheelMotor, MotorType.kBrushless);
    private CANSparkMax indexingMotor = new CANSparkMax(Constants.Shooter.indexingMotor, MotorType.kBrushless);
    private DigitalInput shooterBeamBreak = new DigitalInput(Constants.Shooter.shooterBeamBreakDIOPort);

    // Constructor
    public ArmSubsystem() {
        
        leftLeaderFlywheelMotor.setInverted(Constants.Shooter.leftLeaderFlywheelMotorInverted);
        indexingMotor.setInverted(Constants.Shooter.indexingMotorInverted);

        rightFollowerFlywheelMotor.follow(leftLeaderFlywheelMotor, true); 

        

    }


   /* ------------------------------------------------------- *\
   |                        A R M P I D                        |
   \* ------------------------------------------------------- */

    //Values for shooter velocity max & min
    public static final int shooterVelocityMax = 0;
    public static final int shooterVelocityMin = 0;

    private PIDController shooterLeftPID = new PIDController(Constants.Shooter.shooterkP, Constants.Shooter.shooterkI, Constants.Shooter.shooterkD);
    private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.Shooter.shooterFeedkS, Constants.Shooter.shooterFeedkV, Constants.Shooter.shooterFeedkA);
    
    private TrapezoidProfile motionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(shooterVelocityMin, shooterVelocityMax)); //TODO: link constraints
    private TrapezoidProfile.State finalState = new TrapezoidProfile.State(0, 0);
    


    // Hey Brian don't worry about this code, sure it's jank but this is what we know how to do from previous years so let us cook b(￣▽￣)d

    // TODO(from Paul): Question for Brennan/Nate/Brian - Do these methods need to have "Arm" 
    //                  in the name so we can distinguish them from similar functions we'll 
    //                  have for the shooter flywheel (assuming we have functions like setVelocity())?


    public  boolean setArmPosition(float armPosition) {
        // will be passed in with constants from xbox buttons
        // possible positions: kStowPosition, kIntakePosition, kAmpPosition, kClimbingPosition, and the speaker aim position calculated from vision

        // Create a state for the motion profile
        TrapezoidProfile.State currentState = new TrapezoidProfile.State(leftLeaderFlywheelMotor.getEncoder().getPosition(),leftLeaderFlywheelMotor.getEncoder().getVelocity());

        //Set the setpoint of the PID controller
        setSpeed(motionProfile.calculate(0, currentState, finalState).velocity);

        //Set the voltage of the motor using the PID and feedforward controllers
        leftLeaderFlywheelMotor.setVoltage((shooterLeftPID.calculate(getSpeed()) + feedForward.calculate(shooterLeftPID.getSetpoint())));


        return true;
    }

    
    public void setPosition(double position) {
        finalState = new TrapezoidProfile.State(position, 0); // Create a final state for the motion profile
        
    }

    public void setSpeed(double rpm) {
        shooterLeftPID.setSetpoint(rpm); // Set the setpoint of the PID controller
    }

    public void setVoltage(double voltage) {
        leftLeaderFlywheelMotor.setVoltage(voltage); // Set the voltage of the motor
    }

    public double getSpeed() {
        return leftLeaderFlywheelMotor.getEncoder().getVelocity() * Constants.Shooter.CONVERSION_FACTOR; // Get the speed of the motor
    }

    public double getPosition() {
        return leftLeaderFlywheelMotor.getEncoder().getPosition() * Constants.Shooter.CONVERSION_FACTOR; // Get the position of the motor
    }

    public boolean isBusy() {
        return getPosition() < finalState.position + Constants.Shooter.BUSY_TOLERANCE && getPosition() > finalState.position - Constants.Shooter.BUSY_TOLERANCE; // Check if the motor is within the tolerance of the setpoint
    }



    /* ------------------------------------------------------- *\
    |                       S H O O T E R                       |
    \* ------------------------------------------------------- */

    //recieve
    public void recieveNote() {
        //arm is already at kIntakePosition

    }

    //idle flywheels
    public void idleFlywheels() {
        double idleSpeed = 0;
        leftLeaderFlywheelMotor.set(idleSpeed);
    }

    //aim speaker/rev flywheels
    //angle starts at zero, can only increase
    public boolean aimSpeaker(float angle) {
        
        setArmPosition(angle);

        boolean radarLock =! isBusy();

        double flywheelShootSpeed = 0;
        leftLeaderFlywheelMotor.set(flywheelShootSpeed);
        double flywheelSpeed = leftLeaderFlywheelMotor.get(); //gets RPMs?

        double difference = Math.abs(flywheelShootSpeed - flywheelSpeed);
        boolean flywheelRevved = difference < 10.0;

        return (radarLock && flywheelRevved);
    }

    //shoot speaker
    public void shootSpeaker() {
        double loadingSpeed = 0;
        indexingMotor.set(loadingSpeed);
        
    }

   

    //shoot amp
    public void shootAmp() {
        //set flywheels to ___ (between idle and shooting speed)
    }

    
    /* ------------------------------------------------------- *\
    |                       C L I M B E R                       |
    \* ------------------------------------------------------- */
    

    //switch to manual climb
    public void manualClimb(float climbRate) {
        //get climb rate from controller
    }
    

}
