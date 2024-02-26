package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;

/** 
 * Turn the robot to point to the Speaker, and elevate the arm based on the distance to the speaker.
 */
public class AimCommand extends Command {
    

    // TODO Define class variables for the required subsystems: vision (to receive the aiming direction), swerve (to turn the robot), arm (to raise/lower the arm)

    // TODO Define a constructor that takes in the 3 required subsystems

    // TODO write the execute, end, and isFinished methods

    // TODO execute():
    //          1. Get the RotationToSpeaker from the Vision Subsystem
    //          2. Convert the angle to a angular rotation velocity in radians/sec using a simple proportional constant or full PIDF control
    //          3. Call the drive method on the SwerveSubsystem with 0 translation and the rotation just calculated
    //          4. Get the RangeToSpeaker from the Vision Subsystem
    //          5. Convert the range to speaker from a distance to a arm position using a look up table or proportional constant
    //          6. Set the arm position using the appropriate method on the Arm Subsystem

    // TODO end():
    //          Don't think there is anything to do.

    // TODO isFinished():
    //          1. Get the Rotation and range from the Vision Subsystem
    //          2. Do the same conversions as done in execute()
    //          3. return true if both the error in angular velocity and desired arm position is close to zero.
    //                  return false otherwise.
}
