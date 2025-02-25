/* Black Knights Robotics (C) 2025 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.utils.Controller;

public class ClimbCommand extends Command {

    public Controller controller;
    public ClimberSubsystem climberSubsystem;

    public ClimbCommand(Controller controller, ClimberSubsystem climberSubsystem) {
        this.controller = controller;
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        if (controller.dpadDown.getAsBoolean()) {
            climberSubsystem.setLiftMotorPercent(ClimberConstants.LIFT_MOTOR_PERCENTAGE);
        } else if (controller.dpadUp.getAsBoolean()) {
            climberSubsystem.setLiftMotorPercent(-ClimberConstants.LIFT_MOTOR_PERCENTAGE);
        } else if (controller.dpadRight.getAsBoolean()) {
            climberSubsystem.setLockMotorPercent(-ClimberConstants.LOCK_MOTOR_PERCENTAGE);
        } else if (controller.dpadLeft.getAsBoolean()) {
            climberSubsystem.setLockMotorPercent(ClimberConstants.LOCK_MOTOR_PERCENTAGE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setLiftMotorPercent(0.0);
        climberSubsystem.setLiftMotorPercent(0.0);
    }
}
