// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
   * ------------------------------ ------------------------- *\
   * | L E D |
   * \* -------------------------------------------------------
   */

  public static final class aprilTag {
    // Apritag spaces relative to bluespeaker converted to meters
    public static final Translation2d blueSpeaker = new Translation2d(Units.inchesToMeters(-1.5),
        Units.inchesToMeters(218.42));
    public static final Translation2d redSpeaker = new Translation2d(Units.inchesToMeters(652.73),
        Units.inchesToMeters(218.42));
    public static final double speakerHeight = 1.9845;
  }

  public static final class LED {
    public static final int PWMPORT = 0;
    public static final int BUFFERSIZE = 157;

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
    public static final int armEncoderPortA = 0;
    public static final int armEncoderPortB = 1;
    public static final int armEncoderPortS = 2;

    public static final int rightMotorOneID = 19;
    public static final int rightMotorTwoID = 20;
    public static final int leftMotorOneID = 18;
    public static final int leftMotorTwoID = 17;

    public static final double armGearRatio = 72.0 / 15.0 * 56.0 / 20.0 * 58.0 / 10.0;
    public static final double ajustedArmGearRatio = 1 / armGearRatio;

    public static final double armAngleOffset = 0.44688;
    public static final double armTicksPerRevolution = 2048;

    public static final double horizontalArmOffset = 0.0975;

    public static final double armkP = 10; //6.6544; // 3.0565;
    public static final double armkI = 0;
    public static final double armkD = 0; //4.833; // 0;

    // Feed Forward
    public static final double armks = 0.78338;
    public static final double armkg = 0.135;
    public static final double armkv = 5.6111;
    public static final double armka = 0.39689;

    // PID values for arm

    public static final double maxVoltage = 11.5;

    public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(1.25, 1.25);

    // Differnt arm position
    public static class SetPointPositions {
      // 0 is the home position and all the way down
      public static final double kAmpPosition = 0.2587;
      public static final double kStowPosition = -0.0789;
      public static final double kShootFlatPosition = 16.2 / 360.0;
      public static final double kShootWingLinePosition = 0.0238; //0.02745
      public static final double kPodiumLinePosition = -0.00482;   //-0.00382

      // public static final double kBeamFlatPosition = 0;
      
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
      public static final int kLeftSpeaker = 5000;
      public static final int kRightSpeaker = 3000;

      public static final int kidleSpeed = 1000;

      // IDLE
      public static final int kIDLE = 3000;

      public static final double kOverStageFlySpeedL = 3000;
      public static final double kOverStageFlySpeedR = 2800;
      public static final double kRedStageAngle = Units.degreesToRadians(30);
      public static final double kBlueStageAngle = Units.degreesToRadians(-30);
    }

    // CAN ID's for the shooter motors
    public static final int leftLeaderFlywheelMotor = 14;
    public static final int rightLeaderFlywheelMotor = 15;

    public static final boolean kleftMotorInverted = true;
    public static final boolean krightMotorInverted = false;

    public static final int kShooterCurrentLimit = 0;

    public static final double kRightShooterks = 0.040967;
    public static final double kRightShooterkv = 0.0020721;
    public static final double kRightShooterkA = 0.00052497;

    // PID values for shooter
    public static final double kRightShooterkP = 0.00019397;
    public static final double kRightShooterkI = 0;
    public static final double kRightShooterkD = 0;

    public static final double kLeftShooterks = 0.074672;
    public static final double kLeftShooterkv = 0.0021139;
    public static final double kLeftShooterkA = 0.00050288;

    // PID values for shooter
    public static final double kLeftShooterkP = 3.6451E-05;
    public static final double kLeftShooterkI = 0;
    public static final double kLeftShooterkD = 0;

    public static final double maxVoltage = 11.5;
    public static final double maxRPM = 7000;
    public static final double maxAcc = 1500;

    // BeamBreak sensor DIO port for shooter
    public static final int kBeamBreakPort = 0;

    // Even more goofy variables
    public static final double CONVERSION_FACTOR = 1;
    public static final double PID_TOLERANCE = 20;

  }

  // Indexer
  public static final class Indexing {

    public static final int indexingMotor = 16;
    public static final boolean indexingMotorInverted = true;
    public static final int indexingSpeed = 4000;
    public static final int beamBreakIRThreashold = 100;

  }


}
