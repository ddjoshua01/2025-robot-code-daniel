/* Black Knights Robotics (C) 2025 */
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.utils.NetworkTablesUtils;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkFlex liftMotor =
            new SparkFlex(ClimberConstants.LIFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    private final WPI_TalonSRX lockMotor = new WPI_TalonSRX(ClimberConstants.LOCK_MOTOR_ID);

    public void setLiftMotorPercent(double percent) {
        liftMotor.set(percent);
    }

    public void setLockMotorPercent(double percent) {
        lockMotor.set(percent);
    }

    public void periodic() {
        NetworkTablesUtils.getTable("Climber").setEntry("current", liftMotor.getOutputCurrent());
    }
}
