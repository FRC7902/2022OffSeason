package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;

public class ClimberRopeSubsystem extends SubsystemBase{

    private final WPI_VictorSPX ClimberRopeAdjustment = new WPI_VictorSPX(Constants.ClimberRopeConstants.ClimberRopeAdjustment);
    //still need to declare new Spark Max motor for climber rope main

    //Sets output power
    //double power in range [-1, 1]
    public void setPower(double power){
  
      //set output power of motor to input speed
      ClimberRopeAdjustment.set(power);
  
    }
  
    //stops the motor
    public void stopMotors(){
  
      //stops motor
      ClimberRopeAdjustment.stopMotor();
  
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
