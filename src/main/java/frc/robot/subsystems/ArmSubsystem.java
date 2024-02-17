package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    

   /* ------------------------------------------------------- *\
   |                       S H O O T E R                       |
   \* ------------------------------------------------------- */

    private CANSparkMax leftLeaderFlywheelMotor = new CANSparkMax(Constants.Shooter.leftLeaderFlywheelMotor, MotorType.kBrushless);
    private CANSparkMax rightFollowerFlywheelMotor = new CANSparkMax(Constants.Shooter.rightFollowerFlywheelMotor, MotorType.kBrushless);
    private CANSparkMax indexingMotor = new CANSparkMax(Constants.Shooter.indexingMotor, MotorType.kBrushless);

    // ------------------------------------------------------------------------------------
    // TODO: Setup PID Controllers and encoders for (leading) motors. For example:
    //
    // 
    //  GIVEN:  motor is declared as:
    //
    //       CANSparkMax motor = new CANSparkMax(...);
    //
    //  1. Declare class variables for each
    //
    //       PIDController pidController = null;
    //       RelativeEncoder encoder = null;
    //
    //  2. Initialize the variables in the constructor
    //
    //       pidController = motor.getPIDController();
    //       pidController.setP( <some-constant> );
    //       pidController.setI( <some-constant> );
    //       pidController.setD( <some-constant> );
    //       pidController.setI( <some-constant> );
    //       pidController.setFF( <some-constant> );
    //       pidController.setOutputRange( <min>, <max> );
    //
    //       encoder = motor.getEncoder();
    //  
    //  3. Update methods to use the PID Controller by calling the setReference() function:
    //
    //       pidController.setReference( <the-desired-position>, ControlType.kPosition);
    //
    //  NOTE: The code for velocity PID Control is very similar, just using ControlType.kVelocity instead
    //        The encoder variable is not actually needed for pid control. The PID Controller on the SparkMAX will get the encoder value directly.
    //        A more detailed example can be found at:
    //        https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
    //
    // ------------------------------------------------------------------------------------


    // ------------------------------------------------------------------------------------
    //
    // Next thing to do is code the sensor interface object. This type of object works for limit switches and Hall effect sensors:
    //        DigitalInput someDigitalIOPortInput = new DigitalInput( <dio-port-number> );
    //
    // ------------------------------------------------------------------------------------
    

    public ArmSubsystem() {

        leftLeaderFlywheelMotor.setInverted(Constants.Shooter.leftLeaderFlywheelMotorInverted);
        indexingMotor.setInverted(Constants.Shooter.indexingMotorInverted);

        rightFollowerFlywheelMotor.follow(leftLeaderFlywheelMotor, true); 
    }


    // Hey Brian don't worry about this code, sure it's jank but this is what we know how to do from previous years so let us cook b(￣▽￣)d

    public  boolean setArmPosition(float armPosition) {
        //will be passed in with constants from xbox buttons
        //possible positions: kStowPosition, kIntakePosition, kAmpPosition, kClimbingPosition, and the speaker aim position calculated from vision
        return true;
    }

    //-----------------AIM/SHOOT-----------------
    

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


        // TODO: Replace with PID Control following example code from: 
        //       https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java

        boolean radarLock = setArmPosition(angle);

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

    //--------------CLIMB--------------
    

    //switch to manual climb
    public void manualClimb(float climbRate) {
        //get climb rate from controller
    }
    

}
