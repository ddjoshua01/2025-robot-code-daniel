/* Black Knights Robotics (C) 2025 */
package org.blackknights.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.blackknights.subsystems.IntakeSubsystem;
import org.blackknights.utils.ConfigManager;

/** Command to intake and outtake */
public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final IntakeMode mode;

    private double startTime;

    /**
     * Create a new intake command
     *
     * @param intakeSubsystem The instance of {@link IntakeSubsystem}
     * @param mode The intake mode ({@link IntakeMode})
     */
    public IntakeCommand(IntakeSubsystem intakeSubsystem, IntakeMode mode) {
        this.intakeSubsystem = intakeSubsystem;
        this.mode = mode;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        this.startTime = Timer.getFPGATimestamp() * 1000;
    }

    @Override
    public void execute() {
        switch (mode) {
            case INTAKE:
                {
                    intakeSubsystem.setVoltage(
                            ConfigManager.getInstance().get("intake_speed", 8.0));
                    break;
                }
            case OUTTAKE:
                {
                    if (Timer.getFPGATimestamp() * 1000 - this.startTime
                            > ConfigManager.getInstance().get("outtake_wait_time_ms", 40)) {
                        intakeSubsystem.setVoltage(
                                ConfigManager.getInstance().get("outtake_speed", -8.0));
                    }
                    break;
                }
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return (mode.equals(IntakeMode.INTAKE) && intakeSubsystem.getLinebreak())
                || (mode.equals(IntakeMode.OUTTAKE)
                        && !intakeSubsystem.getLinebreak()
                        && Timer.getFPGATimestamp() * 1000 - this.startTime
                                > ConfigManager.getInstance().get("outtaking_time_ms", 40));
    }

    /** Enum of the different intake modes */
    public enum IntakeMode {
        INTAKE,
        OUTTAKE
    }
}
