package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TransferConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class TransferSubsystem extends SubsystemBase{
    
    private final WPI_VictorSPX transferController = new WPI_VictorSPX(Constants.TransferConstants.VerticalTransferCAN);
    public TransferSubsystem(){
        transferController.setInverted(true);

        transferController.configOpenloopRamp(Constants.TransferConstants.VerticalTransferRampTimeInSeconds);


    }

    public void setTransferPower(double power){
        transferController.set(power);
    }

    public void stopTransfer(){
        transferController.set(0);
    }
}
