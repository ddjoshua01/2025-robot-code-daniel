/* Black Knights Robotics (C) 2025 */
package org.blackknights.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.blackknights.constants.AlignConstants;
import org.blackknights.framework.Odometry;
import org.blackknights.subsystems.SwerveSubsystem;
import org.blackknights.utils.ConfigManager;
import org.blackknights.utils.NetworkTablesUtils;

/** Align the robot in fieldspace */
public class AlignCommand extends Command {
    private static final Logger LOGGER = LogManager.getLogger();
    private final SwerveSubsystem swerveSubsystem;

    private final ProfiledPIDController rotationPid =
            new ProfiledPIDController(
                    AlignConstants.ROTATION_P,
                    AlignConstants.ROTATION_I,
                    AlignConstants.ROTATION_D,
                    AlignConstants.ROTATION_CONSTRAINTS);

    private TrapezoidProfile xProfile;
    private TrapezoidProfile yProfile;

    private final Odometry odometry = Odometry.getInstance();
    private final ConfigManager configManager = ConfigManager.getInstance();

    private final String profile;
    private final boolean stopWhenFinished;

    private final Supplier<Pose2d> pose2dSupplier;

    private final NetworkTablesUtils debug = NetworkTablesUtils.getTable("debug");

    private Pose2d targetPos;

    private double timeSenseFinished = -1;
    private boolean doUpdate = true;

    private double distToTarget = Double.MAX_VALUE;

    /**
     * Align to a fieldspace position with odometry
     *
     * @param swerveSubsystem The instance of swerve subsystem // * @param controller The primary
     *     driving {@link edu.wpi.first.wpilibj.XboxController}, used for driver to override vision
     * @param poseSupplier A {@link Supplier<Pose2d>} for poses
     * @param stopWhenFinished Weather to stop swerve or not when the command is complete, set to
     *     false if you are doing multiple paths in a row
     * @param profile The tuning profile to use, generates separate entries in {@link ConfigManager}
     *     for tolerances and trapezoid tuning (DON'T spell it wrong unless you want 10 extra
     *     useless values in cfg manager!!!)
     */
    public AlignCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Pose2d> poseSupplier,
            boolean stopWhenFinished,
            String profile) {
        this.swerveSubsystem = swerveSubsystem;
        this.pose2dSupplier = poseSupplier;
        this.stopWhenFinished = stopWhenFinished;
        this.profile = profile;

        LOGGER.debug("Created new align command with '{}' profile", this.profile);

        Pose3d robotPose = Odometry.getInstance().getRobotPose();
        this.rotationPid.enableContinuousInput(-Math.PI, Math.PI);

        this.rotationPid.reset(
                robotPose.getRotation().getZ(),
                swerveSubsystem.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond);

        this.rotationPid.setGoal(robotPose.getRotation().getZ());
        this.rotationPid.calculate(robotPose.getRotation().getZ());

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        this.targetPos = pose2dSupplier.get();
        this.timeSenseFinished = -1;
        this.doUpdate = true;
        this.distToTarget = Double.MAX_VALUE;

        LOGGER.info("Initializing AlignCommand");
        Pose3d robotPose = odometry.getRobotPose();

        // rot PID updates
        this.rotationPid.setP(configManager.get("align_rot_p", 7.3));
        this.rotationPid.setD(configManager.get("align_rot_d", 0.5));
        this.rotationPid.setI(configManager.get("align_rot_i", 0.0));

        this.xProfile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                configManager.get(
                                        String.format("align_%s_x_max_vel_m", this.profile), 3.0),
                                configManager.get(
                                        String.format("align_%s_x_max_accel_mps", this.profile),
                                        2.5)));

        this.yProfile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                configManager.get(
                                        String.format("align_%s_y_max_vel_m", this.profile), 3.0),
                                configManager.get(
                                        String.format("align_%s_y_max_accel_mps", this.profile),
                                        2.5)));

        this.rotationPid.setConstraints(
                new TrapezoidProfile.Constraints(
                        Math.toRadians(
                                configManager.get(
                                        String.format("align_%s_rot_max_vel_deg", this.profile),
                                        360)),
                        Math.toRadians(
                                configManager.get(
                                        String.format("align_%s_rot_max_accel_degps", this.profile),
                                        360))));

        this.rotationPid.setTolerance(
                Math.toRadians(
                        configManager.get(
                                String.format("align_%s_rotation_tolerance", this.profile), 1)));

        // Reset All pids
        this.rotationPid.reset(
                robotPose.getRotation().getZ(),
                swerveSubsystem.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond);

        this.rotationPid.setGoal(targetPos.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose3d robotPose = odometry.getRobotPose();

        this.distToTarget =
                Math.pow(robotPose.getX() - targetPos.getX(), 2)
                        + Math.pow(robotPose.getY() - targetPos.getY(), 2);

        double xAxisCalc =
                this.xProfile.calculate(
                                0,
                                new TrapezoidProfile.State(
                                        robotPose.getX(),
                                        swerveSubsystem.getFieldRelativeChassisSpeeds()
                                                .vxMetersPerSecond),
                                new TrapezoidProfile.State(targetPos.getX(), 0))
                        .velocity;
        double yAxisCalc =
                this.yProfile.calculate(
                                0,
                                new TrapezoidProfile.State(
                                        robotPose.getY(),
                                        swerveSubsystem.getFieldRelativeChassisSpeeds()
                                                .vyMetersPerSecond),
                                new TrapezoidProfile.State(targetPos.getY(), 0))
                        .velocity;
        double rotationPidCalc = this.rotationPid.calculate(robotPose.getRotation().getZ());

        debug.setEntry("Dist to target (Error)", distToTarget);

        debug.setEntry("X Error", Math.abs(robotPose.getX() - targetPos.getX()));
        debug.setEntry("Y Error", Math.abs(robotPose.getY() - targetPos.getY()));
        debug.setEntry("Rot Pid Error", this.rotationPid.getPositionError());

        debug.setEntry("Rot Pid setpoint", this.rotationPid.atSetpoint());
        debug.setEntry("Rot Pid goal", this.rotationPid.atGoal());
        debug.setEntry("Robot rotation: ", robotPose.getRotation().getZ());
        debug.setEntry("Rot setpoint", this.rotationPid.getSetpoint().position);

        debug.setEntry(
                "Rot diff",
                Math.abs(
                        Math.abs(this.targetPos.getRotation().getRadians())
                                - Math.abs(odometry.getRobotPose().getRotation().getZ())));

        debug.setEntry("Xms", xAxisCalc);
        debug.setEntry("Yms", yAxisCalc);
        debug.setEntry("Rrads", rotationPidCalc);

        this.debug.setArrayEntry(
                "target_pose",
                new double[] {
                    this.targetPos.getX(),
                    this.targetPos.getY(),
                    this.targetPos.getRotation().getRadians()
                });

        swerveSubsystem.drive(xAxisCalc, yAxisCalc, 0.0, true, !stopWhenFinished, true);

        if (checkAtGoal() && doUpdate) {
            LOGGER.info("Hit goal, waiting for time to expire");
            this.timeSenseFinished = Timer.getFPGATimestamp() * 1000;
            this.doUpdate = false;
        }
    }

    @Override
    public boolean isFinished() {
        return checkAtGoal()
                && Timer.getFPGATimestamp() * 1000 - this.timeSenseFinished
                        > configManager.get(
                                String.format("align_%s_finish_time", this.profile), 200.0);
    }

    @Override
    public void end(boolean interrupted) {
        //        if (stopWhenFinished) swerveSubsystem.zeroVoltage();
        //        else swerveSubsystem.drive(this.finalX, this.finalY, 0.0, true, true, true);
        //        LOGGER.info("Final commanded speeds: {} {}", this.finalX, this.finalY);
    }

    private boolean checkAtGoal() {
        return distToTarget
                        <= configManager.get(
                                String.format("align_%s_pos_dist_tol", this.profile), 0)
                && rotationPid.atGoal()
                && MathUtil.isNear(
                        configManager.get(
                                String.format("align_%s_x_target_end_vel", this.profile), 0.0),
                        swerveSubsystem.getFieldRelativeChassisSpeeds().vxMetersPerSecond,
                        configManager.get("align_vel_tol", 0.0))
                && MathUtil.isNear(
                        configManager.get(
                                String.format("align_%s_y_target_end_vel", this.profile), 0.0),
                        swerveSubsystem.getFieldRelativeChassisSpeeds().vyMetersPerSecond,
                        configManager.get("align_vel_tol", 0.0));
    }
}
