package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;



public class TransferSubsystem extends SubsystemBase{
    
    private final WPI_VictorSPX transferController = new WPI_VictorSPX(Constants.TransferConstants.TransferVerticalCAN);

    public TransferSubsystem(){
        transferController.setInverted(true);
    }


  //Sets output power
  //double power in range [-1, 1]
  public void setTransferPower(double power){

    //set output power of motor to input speed
    transferController.set(power);

  }

  public void stopTransfer(){

    transferController.set(0);

  }

  public void setSpeed(double speed){
    transferController.set(speed);
    //speed > 0, transferring up
    //speed < 0, transferring down
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // For example, printing values to smart dashboard or shuffleboard

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
