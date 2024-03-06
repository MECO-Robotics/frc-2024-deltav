package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexingSubsystem extends SubsystemBase{
    
    DigitalInput indexerBeambreak = new DigitalInput(Constants.Shooter.shooterBeamBreakDIOPort);

    private CANSparkMax indexingMotor = new CANSparkMax(Constants.Indexing.indexingMotor, MotorType.kBrushless);

    public IndexingSubsystem(){
        indexingMotor.setInverted(Constants.Indexing.indexingMotorInverted);
    }

    public void setIndexingVoltage(double indexingVoltage){
        indexingMotor.setVoltage(indexingVoltage);
    }

    public boolean isNoteAquired(){
        return indexerBeambreak.get();
    }

}
