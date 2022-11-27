package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/** An example command that uses an example subsystem. */

public class TimedDriveWithSpeed extends CommandBase {

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final DriveSubsystem m_DriveSubsystem;
    Timer timer = new Timer();
    double time;
    double speed;

    /**
     * 
     * Creates a new ExampleCommand.
     *
     * 
     * 
     * @param subsystem The subsystem used by this command.
     * 
     */

    public TimedDriveWithSpeed(DriveSubsystem driveSubsystem, double speed, double time) {

        m_DriveSubsystem = driveSubsystem;
        this.speed = speed;
        this.time = time;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Stop everything
        m_DriveSubsystem.driveArcade(0, 0);

        // Reset and restart timer
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (!isFinished()){
            m_DriveSubsystem.driveArcade(speed, 0);
        }
        else {
            m_DriveSubsystem.driveArcade(0, 0);
        }
    }


    @Override
    public boolean isFinished(){

        return timer.get() >= time;
    }


}