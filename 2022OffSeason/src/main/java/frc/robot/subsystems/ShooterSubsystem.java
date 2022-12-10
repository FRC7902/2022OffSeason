package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {}

    VictorSPX leaderMotor = new VictorSPX(Constants.ShooterConstants.ShooterLeaderCAN);
    VictorSPX followerMotor = new VictorSPX(Constants.ShooterConstants.ShooterFollowerCAN);
  @Override

  public void periodic() {
    // This method will be called once per scheduler run

    followerMotor.follow(leaderMotor);

    leaderMotor.setInverted(false);
    followerMotor.setInverted((InvertType.FollowMaster));

  }

  public void shoot(){
    leaderMotor.set(ControlMode.PercentOutput, 1.0);
  }
  public void load(){
    leaderMotor.set(ControlMode.PercentOutput, 0.5);
  }
  public void stop(){
    leaderMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}