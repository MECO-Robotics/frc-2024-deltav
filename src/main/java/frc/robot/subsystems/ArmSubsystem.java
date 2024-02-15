package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    

   /* ------------------------------------------------------- *\
   |                       S H O O T E R                       |
   \* ------------------------------------------------------- */

    private CANSparkMax leftLeaderFlywheelMotor = new CANSparkMax(Constants.Shooter.leftLeaderFlywheelMotor, MotorType.kBrushless);
    private CANSparkMax rightFollowerFlywheelMotor = new CANSparkMax(Constants.Shooter.rightFollowerFlywheelMotor, MotorType.kBrushless);
    private CANSparkMax indexingMotor = new CANSparkMax(Constants.Shooter.indexingMotor, MotorType.kBrushless);



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
