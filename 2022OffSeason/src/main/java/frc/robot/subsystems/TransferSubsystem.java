package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;



public class TransferSubsystem extends SubsystemBase{
    
    private final WPI_VictorSPX transferController = new WPI_VictorSPX(Constants.TransferConstants.TransferVertical);

  //Sets output power
  //double power in range [-1, 1]
  public void setPower(double power){

    //set output power of motor to input speed
    transferController.set(power);

  }

  //stops the motor
  public void stopMotors(){

    //stops motor
    transferController.stopMotor();

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
