/* Black Knights Robotics (C) 2025 */
package frc.robot.framework;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.ScoringConstants;
import frc.robot.utils.ConfigManager;
import frc.robot.utils.NetworkTablesUtils;
import java.util.*;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/** Keeps a list of future positions to place coral */
public class CoralQueue {
    private static CoralQueue INSTANCE = null;
    private static final Logger LOGGER = LogManager.getLogger();

    private final ArrayList<CoralPosition> coralPositionList = new ArrayList<>();
    private final NetworkTablesUtils NTUtils = NetworkTablesUtils.getTable("CoralQueue");
    private int positionListIndex = 0;
<<<<<<< Updated upstream
    private boolean interrupt = false;
=======

    private final Map<String, Integer> profileIndices = new HashMap<>();
>>>>>>> Stashed changes
    private CoralPosition currentPos = new CoralPosition();
    private String currentProfile = "";

    /** Create a new instance of coral queue */
    protected CoralQueue() {}

    /**
     * Get the instance of coral queue, or create a new instance if one does not exist
     *
     * @return The instance of {@link CoralQueue}
     */
    public static CoralQueue getInstance() {
        if (INSTANCE == null) INSTANCE = new CoralQueue();
        return INSTANCE;
    }

    /**
     * Add position string to queue. No spaces, separated by commas. (e.g. '10L2,1L3,2L2,1L1,5L4').
     *
     * @param posStrList The list of pose strings
     */
    public void listToQueue(String posStrList) {
        String[] posList = posStrList.split(",");
        for (String i : posList) {
            addPosition(i);
        }
    }

    /**
     * Return next reef position in the queue
     *
     * @return Get the current coral queue position
     */
    public CoralPosition getCurrentPosition() {
        if (!coralPositionList.isEmpty() && !this.interrupt) {
            this.currentPos = coralPositionList.get(positionListIndex);
            return this.currentPos;
        } else if (this.interrupt) {
            this.interrupt = false;
            return this.currentPos;
        } else {
            return new CoralPosition();
        }
    }

    /**
     * Return the next position to goto and increment the position in queue
     *
     * @return The next position
     */
    public CoralPosition getNext() {
        CoralPosition pos = this.getCurrentPosition();
        this.stepForwards();
        return pos;
    }

    /** Step backwards in the queue */
    public void stepBackwards() {
        this.positionListIndex -= 1;
        if (positionListIndex < 0) {
            positionListIndex = 0;
        }

        if (!currentProfile.isEmpty()) {
            profileIndices.put(currentProfile, positionListIndex);
        }

        getCurrentPosition();
    }

    /** Step forwards in the queue */
    public void stepForwards() {
        positionListIndex += 1;

        if (positionListIndex > coralPositionList.size() - 1) {
            positionListIndex = coralPositionList.size() - 1;
        }

        if (!currentProfile.isEmpty()) {
            profileIndices.put(currentProfile, positionListIndex);
        }

        getCurrentPosition();
    }

    /**
     * Interrupt the queue with a different position, the resume afterward
     *
     * @param position The position to inserted
     */
    public void interruptQueue(CoralPosition position) {
        this.stepBackwards();
        this.interrupt = true;
        this.currentPos = position;
    }

    /**
     * Turn a string into a {@link CoralPosition}
     *
     * @param posStr The string position
     * @return The corresponding {@link CoralPosition}
     */
    public static CoralPosition getCoralPosition(String posStr) {
        if (posStr == null || posStr.isEmpty()) {
            return null;
        }

        DriverStation.refreshData();
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

        int splitIdx = posStr.length() - 2;
        String heightString = posStr.substring(splitIdx);
        String posString = posStr.substring(0, splitIdx);

        int posIdx = Math.max(Math.min(Integer.parseInt(posString) - 1, 23), 0);

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            posIdx += 12;
        }

        LOGGER.debug("CoralQ: posIdx = {}, heightString = {}", posIdx, heightString);

        return new CoralPosition(
                posStr,
                new Pose2d(
                        ScoringConstants.CORAL_POSITIONS[posIdx].getX(),
                        ScoringConstants.CORAL_POSITIONS[posIdx].getY(),
                        ScoringConstants.CORAL_POSITIONS[posIdx].getRotation()),
                ScoringConstants.ScoringHeights.valueOf(heightString.toUpperCase()));
    }

    /**
     * Convert reef position IDs to heights and positions.
     *
     * @param posStr The pose string to be turned into a {@link CoralPosition}
     */
    public void addPosition(String posStr) {
        if (posStr == null || posStr.isEmpty()) {
            return;
        }

        CoralPosition pos = getCoralPosition(posStr);
        if (pos == null) {
            LOGGER.warn("getCoralPosition() returned null pose");
            return;
        }

        coralPositionList.add(pos);
    }

    /** Clear the queue. */
    public void clearList() {
        coralPositionList.clear();
        this.positionListIndex = 0;
    }

    /**
     * Input position string and add it to queue via network tables. No spaces, separated by commas.
     * (e.g. 10L2,1L3,2L2,1L1).
     */
    public void loadQueueFromNT() {
        this.clearList();
        this.listToQueue(ConfigManager.getInstance().get("Coral_Queue", "10L2,11L1"));
    }

    /**
     * Load unique coral profiles from the button board, and save each profile's indices.
     *
     * @param profile
     */
    public void loadQueueFromInput(String profile) {
        String[] profileChosen = ScoringConstants.PROFILES.get(profile);

        if (profileChosen == null) {
            return;
        }

        ConfigManager.getInstance().set("Coral_Queue", String.join(",", profileChosen));

        for (String i : profileChosen) {
            addPosition(i);
        }

        if (!currentProfile.isEmpty() && !currentProfile.equals(profile)) {
            profileIndices.put(currentProfile, positionListIndex);
        }

        clearList();

        for (String i : profileChosen) {
            addPosition(i);
        }

        currentProfile = profile;

        if (profileIndices.containsKey(profile)) {
            int savedIndex = profileIndices.get(profile);
            positionListIndex = Math.min(savedIndex, coralPositionList.size() - 1);
        } else {
            positionListIndex = 0;
            profileIndices.put(profile, positionListIndex);
        }
    }

    /** Runs every 20ms to update NT position */
    public void periodic() {
        NTUtils.setArrayEntry(
                "Current Reef Pose",
                new double[] {
                    this.currentPos.getPose().getX(),
                    this.currentPos.getPose().getY(),
                    this.currentPos.getPose().getRotation().getRadians()
                });

        NTUtils.setEntry("Current Reef Pose Name", currentPos.toString());
        NTUtils.setArrayEntry("Current Reef Height", currentPos.getBooleanHeights());
        NTUtils.setEntry("Position Index", positionListIndex);
    }

    /**
     * Represents a position to score including the String id, {@link Pose2d}, and {@link
     * ScoringConstants.ScoringHeights}
     */
    public static class CoralPosition {
        private final String stringId;
        private final Pose2d pose;
        private final ScoringConstants.ScoringHeights height;

        /**
         * Create a new coral position
         *
         * @param stringId The string ID for the pose
         * @param pose A {@link Pose2d} for the soring position
         * @param height The target height
         */
        public CoralPosition(String stringId, Pose2d pose, ScoringConstants.ScoringHeights height) {
            this.stringId = stringId;
            this.pose = pose;
            this.height = height;
        }

        /** Create an empty coral position */
        public CoralPosition() {
            this.stringId = "";
            this.pose = new Pose2d();
            this.height = ScoringConstants.ScoringHeights.L1;
        }

        /**
         * Get the scoring {@link Pose2d}
         *
         * @return The scoring pose
         */
        public Pose2d getPose() {
            return this.pose;
        }

        /**
         * Get the height
         *
         * @return The elevator height as a {@link ScoringConstants.ScoringHeights}
         */
        public ScoringConstants.ScoringHeights getHeight() {
            return this.height;
        }

        /**
         * Return the pose as a double array
         *
         * @return The pose as a double array with a len of 3 (x, y, rads)
         */
        public double[] getPoseAsDoubleArray() {
            Pose2d pose = getPose();
            return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
        }

        public boolean[] getBooleanHeights() {
            boolean[] heights = new boolean[4];

            int index = Integer.parseInt(String.valueOf(this.height.toString().charAt(1))) - 1;
            heights[index] = true;
            return heights;
        }

        @Override
        public String toString() {
            return String.format(
                    "CoralPosition(stringId: %s, pose: %s, height: %s)",
                    this.stringId, this.pose.toString(), this.height);
        }

        @Override
        public int hashCode() {
            return Objects.hash(this.stringId, this.pose, this.height);
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            CoralPosition that = (CoralPosition) obj;
            return Objects.equals(this.stringId, that.stringId)
                    && Objects.equals(this.pose, that.pose)
                    && this.height == that.height;
        }
    }
}
