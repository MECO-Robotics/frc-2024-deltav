// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CAN;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_SPEED        = 4;
    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double TURN_CONSTANT = 0.75;
  }

  /* ------------------------------------------------------- *\
  |                           A R M                           |
  \* ------------------------------------------------------- */
  //Arm does not include shooter it only the beam and the gearboxes attached to super structure
  public static final class Arm {
    //Creates the CAN id's for the 2 motor arm gearboxes on the super structure
    public static final int rightMotorOne = 0;
    public static final int leftMotorOne = 0;
    public static final int rightMotorTwo = 0;
    public static final int leftMotorTwo = 0;

    public static final boolean rightMotorOneCANIDInverted = true; //IDK if it needs to be inverted
    public static final boolean leftMotorOneCANIDInverted = true;
    public static final boolean rightMotorTwoCANDIDInverted = true;
    public static final boolean leftMotorTwoCANDIDInverted = true;
    //only need 1 current limiting since all the motors should have the same current limit
    public static final int kCurrentLimit = 0; //Sets current limiting

    public static final double kSoftLimitReverse = 0;
    public static final double kSoftLimitForward = 0.0;


    //IDK the gear ratios so these are place holders
    public static final double kArmGearRatio = (1.0 / 25.0) * (28.0 / 50.0) * (16.0 / 64.0); 
    public static final double kPositionFactor =
        kArmGearRatio
            * 2.0
            * Math.PI; // multiply SM value by this number and get arm position in radians
    public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
    public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
    public static final double kArmZeroCosineOffset =
        1.342; // radians to add to converted arm position to get real-world arm position (starts at
    // ~76.9deg angle)
    public static final ArmFeedforward kArmFeedforward =
        new ArmFeedforward(0.0, 3.0, 12.0 / kArmFreeSpeed, 0.0);
    //public static final PIDGains kArmPositionGains = new PIDGains();
    public static final TrapezoidProfile.Constraints kArmMotionConstraint =
        new TrapezoidProfile.Constraints(1.0, 2.0);

    //Differnt arm positions
    public static final double kIntakePosition = 0.0;
    public static final double kClimbingPosition = 0;
    public static final double kAmpPosition = 0;
    public static final double kStowPosition = 0;

    //THe ops are lurking so be aware
  public static final PIDFConfig armPID = new PIDFConfig(0,0,0);    

    //
    
  }

  
  /* ------------------------------------------------------- *\
  |                        I N T A K E                        |
  \* ------------------------------------------------------- */
  public static final class Intake {

    //public static final int kCanId = 0;
    public static final int intakeMotorCANID = 0;

    public static final boolean intakeMotorCANIDInverted = false;

    public static final int kIntakeCurrentLimit = 0;

    //public static final PIDGains kPositionGains = new PIDGains();
    public static final double kPositionTolerance = 0;

    public static final double kIntakePower = 0;

    public static final double kShotFeedTime = 0;

    public static final double kIntakingSpeed = 0;
    public static final double kHandoffSpeed = 0; //(slower than intaking?)

    
  }


  /* ------------------------------------------------------- *\
  |                       S H O O T E R                       |
  \* ------------------------------------------------------- */
  public static final class Shooter{

      //CAN ID's for the shooter motors
      public static final int topFlywheelMotorLeft = 0;
      public static final int topFlywheelMotorRight = 0;
      public static final int bottomFlywheelMotorLeft = 0;
      public static final int bottomFlywheelMotorRight = 0;
      public static final int topIndexingMotor = 0;
      public static final int bottomIndexingMotor = 0;

    CANSparkMax topShooter = new CANSparkMax(topFlywheelMotorLeft, CANSparkMax.MotorType.kBrushless);
    CANSparkMax followermode = new CANSparkMax(topFlywheelMotorRight, CANSparkMax.MotorType.kBrushless); 
    

    
    
    






      public static final boolean topFlywheelMotorInverted = true;
      public static final boolean topIndexingMotorInverted = true;

      public static final int kShooterCurrentLimit = 0;

      //Only needs 1 PID constant for flywheels 
      //Top and bottom in follower mode
      public static final PIDFConfig flywheelPID = new PIDFConfig(0,0,0 );
     
      public static final double IDLEMode = 0;


      
    }



}
