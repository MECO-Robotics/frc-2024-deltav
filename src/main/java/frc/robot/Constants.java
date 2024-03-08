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
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  /*
   * ------------------------------------------------------- *\
   * | L E D |
   * \* -------------------------------------------------------
   */
  public static final class LED {
    public static final int PWMPORT = 0;
    public static final int BUFFERSIZE = 120;

  }

  public static final class Auton {

    public static final PIDFConfig xAutoPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig yAutoPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_SPEED = 4;
    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double TURN_CONSTANT = 0.75;

  }

  /*
   * ------------------------------------------------------- *\
   * | A R M |
   * \* -------------------------------------------------------
   */
  // Arm does not include shooter it only the beam and the gearboxes attached to
  // super structure
  public static final class Arm {
    // Creates the CAN id's for the 2 motor arm gearboxes on the super structure

    // Arm Encoder Port
    public static final int armEncoderPort = 0;

    public static final int rightMotorOneID = 19;
    public static final int rightMotorTwoID = 20;
    public static final int leftMotorOneID = 18;
    public static final int leftMotorTwoID = 17;

    // Feed Forward
    public static final int armks = 0;
    public static final int armkg = 0;
    public static final int armkv = 0;

    // PID values for arm
    public static final double armkP = 0.00005;
    public static final double maxVoltage = 11.5;
    public static final double maxRPM = 7000;
    public static final double maxAcc = 1500;

    public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(1.0, 2.0);

    public static final int kCurrentLimit = 0; // Sets current limiting

    // Differnt arm positions
    public static class SetPointPositions {
      public static final double kIntakePosition = 0.0;
      public static final double kClimbingPosition = 0;
      public static final double kAmpPosition = 0;
      public static final double kStowPosition = 0;

    }

  }

  /*
   * ------------------------------------------------------- *\
   * | I N T A K E |
   * \* -------------------------------------------------------
   */
  public static final class Intake {

    // public static final int kCanId = 0;
    public static final int intakeMotorCANID = 21;

    public static final boolean intakeMotorCANIDInverted = true;

    public static final int kIntakeCurrentLimit = 0;

    // public static final PIDGains kPositionGains = new PIDGains();
    public static final double kPositionTolerance = 0;

    public static final double kIntakePower = 0;

    public static final double kShotFeedTime = 0;

    public static final double kIntakingSpeed = 0;
    public static final double kHandoffSpeed = 0; // (slower than intaking?)

    // BeamBreak sensor DIO port for shooter
    public static final int kBeamBreakSensorPort = 0;

  }

  /*
   * ------------------------------------------------------- *\
   * | S H O O T E R |
   * \* -------------------------------------------------------
   */
  public static final class Shooter {

    public static final class Presets {
      public static final int kLeftSpeaker = 5200;
      public static final int kRightSpeaker = 5100;

      public static final int kidleSpeed = 1000;

      // IDLE
      public static final int kIDLE = 3000;

    }

    // CAN ID's for the shooter motors
    public static final int leftLeaderFlywheelMotor = 14;
    public static final int rightLeaderFlywheelMotor = 15;

    public static final boolean kleftMotorInverted = true;
    public static final boolean krightMotorInverted = false;

    public static final int kShooterCurrentLimit = 0;

    public static final double shooterks = 0;
    public static final double shooterkv = 0;

    // PID values for shooter
    public static final double shooterkP = 0.5;
    public static final double shooterkI = 0;
    public static final double shooterkD = 0;
    public static final double maxVoltage = 11.5;
    public static final double maxRPM = 7000;
    public static final double maxAcc = 1500;

    // BeamBreak sensor DIO port for shooter
    public static final int kBeamBreakPort = 0;


    // Even more goofy variables
    public static final double CONVERSION_FACTOR = 1;
    public static final double BUSY_TOLERANCE = 0;

  }

  // Indexer
  public static final class Indexing {

    public static final int indexingMotor = 16;
    public static final boolean indexingMotorInverted = true;
    public static final int indexingSpeed = 4000;
    public static final int beamBreakIRThreashold = 200;
  }

}
