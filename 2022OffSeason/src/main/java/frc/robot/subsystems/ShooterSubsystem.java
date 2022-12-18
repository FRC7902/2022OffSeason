package frc.robot.subsystems;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class ShooterSubsystem extends SubsystemBase {

  /** Creates a new ExampleSubsystem. */
  // Declaring new motors
  private final WPI_VictorSPX leaderMotor = new WPI_VictorSPX(Constants.ShooterConstants.ShooterLeaderCAN);
  private final WPI_VictorSPX followerMotor = new WPI_VictorSPX(Constants.ShooterConstants.ShooterFollowerCAN);

  // Create a new shooter encoder
  private final Encoder shooterEncoder = new Encoder(Constants.ShooterConstants.encoderA_DIO,
      Constants.ShooterConstants.encoderB_DIO);

  // New bang-bang controller
  BangBangController shooterController = new BangBangController();

  SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(Constants.ShooterConstants.kS,
      Constants.ShooterConstants.kV);

  // ShooterSubsystem constructor
  public ShooterSubsystem() {

    // Follower motor will follow everything leader does
    followerMotor.follow(leaderMotor);

    // Follower follows leader, even in inversion
    leaderMotor.setInverted(false);
    followerMotor.setInverted(InvertType.FollowMaster);

    // Set shooter in coast mode (VERY IMPORTANT FOR BANG-BANG MODE)
    leaderMotor.setNeutralMode(NeutralMode.Coast);
    followerMotor.setNeutralMode(NeutralMode.Coast);

    // Set encoder CPR
    shooterEncoder.setDistancePerPulse(1.0 / Constants.ShooterConstants.encoderCPR);
  }

  // Sets output power
  // double power in [-1,1]
  public void setPower(double power) {

    // Set output power of motor to input speed
    leaderMotor.set(power); // in range [-1,1]
  }

  // Feedback control method for shooter rotations per minute
  public void setRPM_BangBang(double RPM) {

    leaderMotor.set(shooterController.calculate(shooterEncoder.getRate() * 60, RPM));

  }

  // Feedback control method for shooter rotations per minute
  public void setRPM_BangBangFF(double RPM) {

    // Controls a motor with the output of the BangBang controller and a feedforward
    // Shrinks the feedforward slightly to avoid overspeeding the shooter
    leaderMotor.setVoltage(
        0.95 * shooterFF.calculate(RPM)
            + shooterController.calculate(shooterEncoder.getRate() * 60, RPM) * 12.0
    );
  }

  // Stops motor
  public void stopMotors() {
    // Stops motor
    leaderMotor.stopMotor();
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // For example, printing values to smart dashboard or shuffleboard
    SmartDashboard.putNumber("EncoderRPM", shooterEncoder.getRate() * 60);
  }

  @Override
  public void simulationPeriodic() {

    // This method will be called once per scheduler run during simulation

  }

}