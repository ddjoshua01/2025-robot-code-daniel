/* Black Knights Robotics (C) 2025 */
package org.blackknights.framework;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.HashMap;
import java.util.Optional;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.blackknights.constants.DrivetrainConstants;
import org.blackknights.constants.VisionConstants;
import org.blackknights.utils.Camera;
import org.blackknights.utils.ConfigManager;
import org.blackknights.utils.NetworkTablesUtils;

/** System for all odometry related stuff */
public class Odometry {
    /** List of cameras used for vision-based measurements to refine odometry. */
    private HashMap<String, Camera> cameras = new HashMap<>();

    /** Singleton instance of the OdometrySubsystem. */
    private static Odometry INSTANCE = null;

    /** Logger for recording debug and error messages related to odometry subsystem operations. */
    private static final Logger LOGGER = LogManager.getLogger();

    private final NetworkTablesUtils NTTelemetry = NetworkTablesUtils.getTable("Telemetry");
    private final NetworkTablesUtils debug = NetworkTablesUtils.getTable("debug/Odometry");

    private Optional<Pose3d> targetPose = Optional.of(new Pose3d());

    private boolean hasSeenTarget = false;

    /** Pose estimator for the robot, combining wheel-based odometry and vision measurements. */
    private final SwerveDrivePoseEstimator3d poseEstimator =
            new SwerveDrivePoseEstimator3d(
                    DrivetrainConstants.DRIVE_KINEMATICS,
                    new Rotation3d(),
                    new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
                    },
                    new Pose3d(),
                    new Matrix<>(
                            Nat.N4(),
                            Nat.N1(),
                            new double[] {
                                ConfigManager.getInstance()
                                        .get("odom_wheel_trust", VisionConstants.WHEEL_TRUST),
                                ConfigManager.getInstance()
                                        .get("odom_wheel_trust", VisionConstants.WHEEL_TRUST),
                                ConfigManager.getInstance()
                                        .get("odom_wheel_trust_theta", Math.toRadians(5)),
                                1
                            }),
                    new Matrix<>(
                            Nat.N4(),
                            Nat.N1(),
                            new double[] {
                                ConfigManager.getInstance()
                                        .get("odom_vision_trust", VisionConstants.VISION_TRUST),
                                ConfigManager.getInstance()
                                        .get("odom_vision_trust", VisionConstants.VISION_TRUST),
                                ConfigManager.getInstance()
                                        .get("odom_vision_trust_theta", Math.toRadians(5)),
                                1
                            }));

    private Odometry() {}

    /**
     * Get the instance of Odometry, creating a new one if it doesn't exist
     *
     * @return The instance
     */
    public static Odometry getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Odometry();
        }
        return INSTANCE;
    }

    /**
     * Add a camera to the Odometry for vision-based localization.
     *
     * @param camera The {@link Camera} to add to the system.
     */
    public void addCamera(Camera camera) {
        this.cameras.put(camera.getName(), camera);
    }

    public SwerveDrivePoseEstimator3d getPoseEstimator() {
        return this.poseEstimator;
    }

    /**
     * Get a camera by name
     *
     * @param name The name of the camera
     * @return Either an empty optional if the camera does not exist, or the camera
     */
    public Optional<Camera> getCamera(String name) {
        if (!this.cameras.containsKey(name)) {
            return Optional.empty();
        }

        return Optional.of(this.cameras.get(name));
    }

    /**
     * Get the current estimated pose of the robot
     *
     * @return A {@link Pose3d} of the robot
     */
    public Pose3d getRobotPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose3d pose) {
        this.poseEstimator.resetPosition(
                new Rotation3d(),
                new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
                },
                pose);
    }

    /**
     * @return
     */
    public Optional<Pose3d> getTargetPose() {
        return this.targetPose;
    }

    /**
     * @return Whether the robot has seen a target at any point since the last reset
     */
    public boolean hasSeenTarget() {
        return this.hasSeenTarget;
    }

    /**
     * Add odometry data from wheels
     *
     * @param gyroRotation A {@link Rotation3d} from the gyro
     * @param swerveModulePositions 4 {@link SwerveModulePosition} objects
     */
    public void addWheelOdometry(
            Rotation3d gyroRotation, SwerveModulePosition[] swerveModulePositions) {
        if (swerveModulePositions.length != 4) {
            LOGGER.error("Wrong length for module positions");
            return;
        }

        this.poseEstimator.update(gyroRotation, swerveModulePositions);
    }

    public void periodic() {
        NTTelemetry.setArrayEntry(
                "Pose",
                new double[] {
                    this.getRobotPose().getX(),
                    this.getRobotPose().getY(),
                    this.getRobotPose().getRotation().getZ()
                });
        for (Camera c : this.cameras.values()) {
            Optional<Pose3d> pose = c.getPoseFieldSpace(this.getRobotPose());
            debug.setEntry(String.format("%s/enabled", c.getName()), c.isEnabled());
            if (pose.isPresent() && c.isEnabled()) {
                double dist =
                        Math.sqrt(
                                Math.pow(c.getTargetPose().getX(), 2)
                                        + Math.pow(c.getTargetPose().getY(), 2));

                debug.setEntry(String.format("%s/dist_to_target", c.getName()), dist);
                debug.setArrayEntry(
                        String.format("%s/pose", c.getName()),
                        new double[] {
                            pose.get().getX(), pose.get().getY(), pose.get().getRotation().getZ()
                        });

                if (dist <= ConfigManager.getInstance().get("vision_cutoff_distance", 3)
                        && dist > ConfigManager.getInstance().get("vision_min_distance", 0.5)) {
                    debug.setEntry(String.format("%s/Adding target", c.getName()), true);

                    this.hasSeenTarget = true;
                    LOGGER.debug("Added vision measurement from `{}`", c.getName());
                    this.targetPose =
                            Optional.of(
                                    new Pose3d(
                                            c.getTargetPose().getX() + this.getRobotPose().getX(),
                                            c.getTargetPose().getY() + this.getRobotPose().getY(),
                                            c.getTargetPose().getZ(),
                                            c.getTargetPose().getRotation()));
                    this.poseEstimator.addVisionMeasurement(pose.get(), c.getTimestamp());
                } else {
                    debug.setEntry(String.format("%s/Adding target", c.getName()), false);
                }
            } else {
                this.targetPose = Optional.empty();
            }
        }
    }
}
