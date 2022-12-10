package frc.robot.commands.transfer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TransferSubsystem;

public class TransferUpCommand extends CommandBase {

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final TransferSubsystem m_transferSubsystem;

    /**
     * 
     * Creates a new ExampleCommand.
     * 
     * @param transferSubsystem The subsystem used by this command.
     * 
     */

    public TransferUpCommand(TransferSubsystem transferSubsystem) {

        m_transferSubsystem = transferSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(transferSubsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        m_transferSubsystem.stopTransfer();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        m_transferSubsystem.setTransferPower(Constants.TransferConstants.VerticalTransferUpPower);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        m_transferSubsystem.stopTransfer();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}