/* Black Knights Robotics (C) 2025 */
package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/** Vision related constants */
public class VisionConstants {
    public static double WHEEL_TRUST = 0.1;
    public static double VISION_TRUST = 0.5;

    public static final Transform3d LEFT_CAM_TRANSFORM =
            new Transform3d(0.235, 0.36, 0.16, new Rotation3d(0.0, 0.0, Math.toRadians(-13)));

    public static final Transform3d RIGHT_CAM_TRANSFORM =
            new Transform3d(0.235, -0.36, 0.16, new Rotation3d(0.0, 0.0, Math.toRadians(13)));
}
