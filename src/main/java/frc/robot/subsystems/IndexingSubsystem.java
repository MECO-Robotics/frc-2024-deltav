package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexingSubsystem extends SubsystemBase {

    AnalogInput indexerBeambreak = new AnalogInput(Constants.Shooter.kBeamBreakPort);

    private CANSparkMax indexingMotor = new CANSparkMax(Constants.Indexing.indexingMotor, MotorType.kBrushless);

    private double sensorValue = 0;


    public IndexingSubsystem() {
        indexingMotor.setInverted(Constants.Indexing.indexingMotorInverted);
        indexerBeambreak.setAccumulatorInitialValue(0);
        indexerBeambreak.setAccumulatorCenter(0);
        indexerBeambreak.resetAccumulator();
    }

    public void setIndexingVoltage(double indexingVoltage) {
        indexingMotor.setVoltage(indexingVoltage);
    }

    public void periodic() {
        SmartDashboard.putNumber("BeamBreak IR value", indexerBeambreak.getValue());
    }   

    public boolean isNoteAquired() {
        return indexerBeambreak.getValue() < Constants.Indexing.beamBreakIRThreashold;
    }

}
