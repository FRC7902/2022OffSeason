package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax elevatorMotor = new CANSparkMax(Constants.ElevatorConstants.ElevatorCAN, MotorType.kBrushless);

  public ElevatorSubsystem() {
      elevatorMotor.setSmartCurrentLimit(Constants.ElevatorConstants.ElevatorCurrentLimit);
  }
  public void setPower(double power){
    elevatorMotor.set(power);
  }
  public void stopMotor(){
      elevatorMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Example message
  }
}
