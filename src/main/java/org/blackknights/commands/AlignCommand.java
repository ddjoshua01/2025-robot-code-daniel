/* Black Knights Robotics (C) 2025 */
package org.blackknights.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.blackknights.framework.Odometry;
import org.blackknights.subsystems.SwerveSubsystem;
import org.blackknights.utils.AlignUtils;
import org.blackknights.utils.ConfigManager;
import org.blackknights.utils.NetworkTablesUtils;

/**
 * Align the robot in fieldspace Config Manager Keys: <br>
 * - align_rot_p - Proportional gain for the rotation PID controller.<br>
 * - align_rot_i - Integral gain for the rotation PID controller.<br>
 * - align_rot_d - Derivative gain for the rotation PID controller.<br>
 * - align_rot_max_vel_deg - Maximum rotational velocity (degrees per second).<br>
 * - align_rot_max_accel_degps - Maximum rotational acceleration (degrees per second squared).<br>
 * - align_rot_tolerance - Tolerance for considering the rotation PID at goal (degrees).<br>
 * - align_[profile]_x_max_vel_m - Maximum velocity for X-axis movement (meters per second).<br>
 * - align_[profile]_x_max_accel_mps - Maximum acceleration for X-axis movement (meters per second
 * squared).<br>
 * - align_[profile]_y_max_vel_m - Maximum velocity for Y-axis movement (meters per second).<br>
 * - align_[profile]_y_max_accel_mps - Maximum acceleration for Y-axis movement (meters per second
 * squared).<br>
 * - align_[profile]_rotation_tolerance - Tolerance for rotational alignment (degrees).<br>
 * - align_[profile]_pos_dist_tol - Position error tolerance before considering the robot aligned
 * (meters).<br>
 * - align_[profile]_vel_tol - Velocity tolerance for stopping criteria (meters per second).<br>
 * - align_[profile]_halfmoon_dist - Distance from the target for the exclusion point(meters).<br>
 * - align_[profile]_halfmoon_tol - The tolerance for the half moon exclusion point (radius of
 * circle) (meters).<br>
 * - align_[profile]_finish_time - Time in milliseconds the robot must stay within tolerances before
 * stopping.<br>
 * - align_trap_t_sec - Time step for trapezoidal motion profile calculations (seconds).<br>
 */
public class AlignCommand extends Command {
    private static final Logger LOGGER = LogManager.getLogger();
    private final SwerveSubsystem swerveSubsystem;

    private TrapezoidProfile distProfile;
    private TrapezoidProfile rotationProfile;

    private SimpleMotorFeedforward rotationFF;

    private final Odometry odometry = Odometry.getInstance();
    private final ConfigManager configManager = ConfigManager.getInstance();

    private final String profile;
    private final boolean stopWhenFinished;
    private final boolean useHalfMoon;

    private final Supplier<Pose2d> pose2dSupplier;

    private final NetworkTablesUtils debug = NetworkTablesUtils.getTable("debug");

    private Pose2d targetPos;

    private double timeSenseFinished = -1;
    private boolean doUpdate = true;

    private double distToTarget = Double.MAX_VALUE;
    private double halfMoonDist = Double.MAX_VALUE;

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
            boolean useHalfMoon,
            String profile) {
        this.swerveSubsystem = swerveSubsystem;
        this.pose2dSupplier = poseSupplier;
        this.stopWhenFinished = stopWhenFinished;
        this.useHalfMoon = useHalfMoon;
        this.profile = profile;

        LOGGER.debug("Created new align command with '{}' profile", this.profile);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        this.targetPos = pose2dSupplier.get();
        this.timeSenseFinished = -1;
        this.doUpdate = true;
        this.distToTarget = Double.MAX_VALUE;
        this.halfMoonDist = Double.MAX_VALUE;

        LOGGER.info("Initializing AlignCommand");

        this.distProfile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                configManager.get(
                                        String.format("align_%s_max_vel_m", this.profile), 3.0),
                                configManager.get(
                                        String.format("align_%s_max_accel_mps", this.profile),
                                        2.5)));

        this.rotationProfile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                Math.toRadians(
                                        configManager.get(
                                                String.format(
                                                        "align_%s_rot_max_vel_deg", this.profile),
                                                360)),
                                Math.toRadians(
                                        configManager.get(
                                                String.format(
                                                        "align_%s_rot_max_accel_degps",
                                                        this.profile),
                                                360))));

        this.rotationFF =
                new SimpleMotorFeedforward(
                        configManager.get("align_rotation_ff_ks", 0.01622),
                        configManager.get("align_rotation_ff_kv", 0.0),
                        0.0,
                        1);
    }

    @Override
    public void execute() {
        Pose3d robotPose = odometry.getRobotPose();
        double d_x = this.targetPos.getX() - robotPose.getX();
        double d_y = this.targetPos.getY() - robotPose.getY();

        this.distToTarget = Math.sqrt(Math.pow(d_x, 2) + Math.pow(d_y, 2));

        Pose2d halfMoonClosePose =
                AlignUtils.getXDistBack(
                        this.targetPos,
                        -configManager.get(
                                String.format("align_%s_halfmoon_dist", this.profile), 0.5));
        this.halfMoonDist =
                Math.sqrt(
                        Math.pow(robotPose.getX() - halfMoonClosePose.getX(), 2)
                                + Math.pow(robotPose.getY() - halfMoonClosePose.getY(), 2));

        double trapCalc =
                -this.distProfile.calculate(
                                configManager.get("align_trap_t_sec", 0.2),
                                new TrapezoidProfile.State(
                                        distToTarget,
                                        -Math.sqrt(
                                                Math.pow(
                                                                swerveSubsystem
                                                                        .getFieldRelativeChassisSpeeds()
                                                                        .vxMetersPerSecond,
                                                                2.0)
                                                        + Math.pow(
                                                                swerveSubsystem
                                                                        .getFieldRelativeChassisSpeeds()
                                                                        .vyMetersPerSecond,
                                                                2.0))),
                                new TrapezoidProfile.State(
                                        0.0,
                                        configManager.get(
                                                String.format(
                                                        "align_%s_ending_vel_mag", this.profile),
                                                1.0)))
                        .velocity;

        double a = Math.atan2(d_y, d_x);

        debug.setEntry("Align/Trap Calc", trapCalc);
        debug.setEntry("Align/Angle", Math.toDegrees(a));
        debug.setEntry(
                "Align/Robot Vel",
                Math.sqrt(
                        Math.pow(
                                        swerveSubsystem.getFieldRelativeChassisSpeeds()
                                                .vxMetersPerSecond,
                                        2.0)
                                + Math.pow(
                                        swerveSubsystem.getRobotRelativeSpeeds().vyMetersPerSecond,
                                        2.0)));

        double xAxisCalc = trapCalc * Math.cos(a);
        double yAxisCalc = trapCalc * Math.sin(a);

        //        if (d_x < 0) {
        //            xAxisCalc = -xAxisCalc;
        //            yAxisCalc = -yAxisCalc;
        //        }

        double errorBound = Math.PI;
        double goal =
                MathUtil.inputModulus(
                        targetPos.getRotation().getRadians() - robotPose.getRotation().getZ(),
                        -errorBound,
                        errorBound);

        double rotCalc =
                this.rotationProfile.calculate(
                                0.1,
                                new TrapezoidProfile.State(
                                        robotPose.getRotation().getZ(),
                                        swerveSubsystem.getFieldRelativeChassisSpeeds()
                                                .omegaRadiansPerSecond),
                                new TrapezoidProfile.State(
                                        goal + robotPose.getRotation().getZ(), 0.0))
                        .velocity;

        rotCalc += this.rotationFF.calculate(rotCalc);

        debug.setEntry("Dist to target (Error)", distToTarget);

        debug.setEntry("X Error", Math.abs(d_x));
        debug.setEntry("Y Error", Math.abs(d_y));

        debug.setEntry("Align/Total time", this.distProfile.totalTime());

        debug.setEntry(
                "Rot diff",
                Math.abs(
                        Math.abs(this.targetPos.getRotation().getRadians())
                                - Math.abs(odometry.getRobotPose().getRotation().getZ())));

        debug.setEntry("Xms", xAxisCalc);
        debug.setEntry("Yms", yAxisCalc);
        debug.setEntry("Rrads", rotCalc);

        this.debug.setArrayEntry(
                "target_pose",
                new double[] {
                    this.targetPos.getX(),
                    this.targetPos.getY(),
                    this.targetPos.getRotation().getRadians()
                });

        if (Math.abs(xAxisCalc)
                        < configManager.get(String.format("align_%s_min_vel", this.profile), 0.002)
                && Math.abs(yAxisCalc)
                        < configManager.get(
                                String.format("align_%s_min_vel", this.profile), 0.002)) {
            swerveSubsystem.zeroVoltage();
        } else {
            swerveSubsystem.drive(xAxisCalc, yAxisCalc, rotCalc, true, false, true);
        }

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
        if (stopWhenFinished) swerveSubsystem.zeroVoltage();
    }

    private boolean checkAtGoal() {
        debug.setEntry(
                "Align/Dist Check",
                distToTarget
                        <= configManager.get(
                                String.format("align_%s_pos_dist_tol", this.profile), 0.0));

        debug.setEntry(
                "Align/Half moon check",
                (!useHalfMoon
                        || halfMoonDist
                                >= configManager.get(
                                        String.format("align_%s_halfmoon_tol", this.profile),
                                        0.0)));

        debug.setEntry(
                "Align/Rotation check",
                Math.abs(
                                Math.abs(Odometry.getInstance().getRobotPose().getRotation().getZ())
                                        - Math.abs(targetPos.getRotation().getRadians()))
                        <= Math.toRadians(
                                ConfigManager.getInstance()
                                        .get(
                                                String.format("align_%s_rot_tol_deg", this.profile),
                                                1.0)));
        debug.setEntry(
                "The value",
                Math.abs(
                        Math.abs(Odometry.getInstance().getRobotPose().getRotation().getZ())
                                - Math.abs(targetPos.getRotation().getRadians())));

        debug.setEntry(
                "The value current",
                Math.abs(Odometry.getInstance().getRobotPose().getRotation().getZ()));

        debug.setEntry("The value target", Math.abs(targetPos.getRotation().getRadians()));

        debug.setEntry(
                "Align/X Vel Check",
                MathUtil.isNear(
                        configManager.get(
                                String.format("align_%s_x_target_end_vel", this.profile), 0.0),
                        swerveSubsystem.getFieldRelativeChassisSpeeds().vxMetersPerSecond,
                        configManager.get(String.format("align_%s_vel_tol", this.profile), 0.0)));

        debug.setEntry(
                "Align/Y Vel Check",
                MathUtil.isNear(
                        configManager.get(
                                String.format("align_%s_y_target_end_vel", this.profile), 0.0),
                        swerveSubsystem.getFieldRelativeChassisSpeeds().vyMetersPerSecond,
                        configManager.get(String.format("align_%s_vel_tol", this.profile), 0.0)));

        return distToTarget
                        <= configManager.get(
                                String.format("align_%s_pos_dist_tol", this.profile), 0.0)
                && Math.abs(
                                Math.abs(Odometry.getInstance().getRobotPose().getRotation().getZ())
                                        - Math.abs(
                                                targetPos.getRotation().getRadians() > Math.PI
                                                        ? targetPos.getRotation().getRadians()
                                                                - Math.PI * 2
                                                        : targetPos.getRotation().getRadians()))
                        <= Math.toRadians(
                                ConfigManager.getInstance()
                                        .get(
                                                String.format("align_%s_rot_tol_deg", this.profile),
                                                1.0))
                && (!useHalfMoon
                        || halfMoonDist
                                >= configManager.get(
                                        String.format("align_%s_halfmoon_tol", this.profile), 0.0))
                && (!stopWhenFinished
                        || MathUtil.isNear(
                                configManager.get(
                                        String.format("align_%s_ending_vel_mag", this.profile),
                                        1.0),
                                Math.sqrt(
                                        Math.pow(
                                                        swerveSubsystem
                                                                .getFieldRelativeChassisSpeeds()
                                                                .vxMetersPerSecond,
                                                        2)
                                                + Math.pow(
                                                        swerveSubsystem
                                                                .getFieldRelativeChassisSpeeds()
                                                                .vxMetersPerSecond,
                                                        2)),
                                configManager.get(
                                        String.format("align_%s_vel_tol", this.profile), 0.0)));
    }
}
