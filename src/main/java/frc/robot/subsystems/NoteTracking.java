
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NoteAlignConstants;
import frc.robot.LimelightHelpers;

public class NoteTracking extends SubsystemBase {
 
  private String m_limelightName;
  private double tx;
  private boolean tv;
  private PIDController m_pid;
  public boolean active;
  public double correction;

  public NoteTracking() {
    m_limelightName = "limelight";

    m_pid = new PIDController(NoteAlignConstants.kP, NoteAlignConstants.kI, NoteAlignConstants.kD);
    m_pid.setTolerance(NoteAlignConstants.kTolerance);
    m_pid.setSetpoint(0);

    active = false;
  }

  @Override
  public void periodic() {
    tv = LimelightHelpers.getTV(m_limelightName);

    if (active && tv) {
      tx = LimelightHelpers.getTX(m_limelightName);

      correction = m_pid.calculate(tx);
    }
    if (!active) {
      correction = 0;
    }
   }

   public void set(boolean state) {
    active = state;
   }

   public boolean get() {
    return active;
   }

   public Command setCmd(boolean state) {
    return runOnce(() -> set(state));
   }

   public void toggle() {
    active = !active;
   }

   public Command toggleCmd() {
    return runOnce(() -> toggle());
   }
}
