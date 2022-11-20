package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public ShooterSubsystem() {}

  private final VictorSPX leaderMotor = new VictorSPX(Constants.ShooterConstants.ShooterLeaderCAN);
  private final VictorSPX followerMotor = new VictorSPX(Constants.ShooterConstants.ShooterFollowerCAN);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}