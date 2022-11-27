package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class ShooterSubsystem extends SubsystemBase {

  /** Creates a new ExampleSubsystem. */
  // Declaring new motors
  private final WPI_VictorSPX leaderMotor = new WPI_VictorSPX(Constants.ShooterConstants.ShooterLeaderCAN);
  private final WPI_VictorSPX followerMotor = new WPI_VictorSPX(Constants.ShooterConstants.ShooterFollowerCAN);

  public ShooterSubsystem() {

    // Follower motor will follow everything leader does
    followerMotor.follow(leaderMotor);

    // Follower follows leader, even in inversion
    leaderMotor.setInverted(false);
    followerMotor.setInverted(InvertType.FollowMaster);
  }
   

  // Sets output power
  // double power in [-1,1]
  public void setPower (double power){
    
    // Set output power of motor to input speed
    leaderMotor.set(power); // in range [-1,1]
  }

  // Stops motor
  public void stopMotors (){
    // Stops motor
    leaderMotor.stopMotor();
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