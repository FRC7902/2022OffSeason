package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants;

public class PneumaticSubsystem {

    private final Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

    private final DoubleSolenoid intakeDeploymentSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            Constants.IntakeConstants.IntakeDeploymentForwardPCMChannel,
            Constants.IntakeConstants.IntakeDeploymentReversePCMChannel);

    public PneumaticSubsystem() {

        pcmCompressor.enableDigital();
        pcmCompressor.disable();

        boolean enabled = pcmCompressor.enabled();
        boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
        double current = pcmCompressor.getCurrent();

    }

    public void extendIntake() {

        intakeDeploymentSolenoid.set(Value.kForward);

    }

    public void retractIntake(){

        intakeDeploymentSolenoid.set(Value.kReverse);
    }

}
