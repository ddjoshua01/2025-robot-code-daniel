/* Black Knights Robotics (C) 2025 */
package org.blackknights.framework;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.blackknights.constants.ScoringConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class CoralQueueTest {
    private CoralQueue coralQueue;

    @BeforeEach
    void setUp() {
        coralQueue = new CoralQueue();
    }

    @Test
    void testListToQueue() {
        coralQueue.loadProfile(CoralQueue.CoralQueueProfile.fromString("11L4,1L1,3L3"));
        CoralQueue.CoralPosition expected =
                new CoralQueue.CoralPosition(
                        "11L4",
                        ScoringConstants.CORAL_POSITIONS[10],
                        ScoringConstants.ScoringHeights.L4,
                        ScoringConstants.ScoringSides.RIGHT);

        assertEquals(expected, coralQueue.getNext());

        expected =
                new CoralQueue.CoralPosition(
                        "1L1",
                        ScoringConstants.CORAL_POSITIONS[0],
                        ScoringConstants.ScoringHeights.L1,
                        ScoringConstants.ScoringSides.RIGHT);

        assertEquals(expected, coralQueue.getNext());

        expected =
                new CoralQueue.CoralPosition(
                        "3L3",
                        ScoringConstants.CORAL_POSITIONS[2],
                        ScoringConstants.ScoringHeights.L3,
                        ScoringConstants.ScoringSides.RIGHT);

        assertEquals(expected, coralQueue.getNext());
    }

    @Test
    void testSkipNextValue() {
        coralQueue.loadProfile(CoralQueue.CoralQueueProfile.fromString("10L4,1L1"));
        coralQueue.stepForwards();
        CoralQueue.CoralPosition expected =
                new CoralQueue.CoralPosition(
                        "1L1",
                        ScoringConstants.CORAL_POSITIONS[0],
                        ScoringConstants.ScoringHeights.L1,
                        ScoringConstants.ScoringSides.RIGHT);
        assertEquals(expected, coralQueue.getNext());
    }

    @Test
    void testBlueAlliancePositionMapping() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue3);
        DriverStationSim.notifyNewData();

        coralQueue.loadProfile(CoralQueue.CoralQueueProfile.fromString("6L1,12L4,9L4"));

        CoralQueue.CoralPosition expected =
                new CoralQueue.CoralPosition(
                        "6L1",
                        ScoringConstants.CORAL_POSITIONS[17],
                        ScoringConstants.ScoringHeights.L1,
                        ScoringConstants.ScoringSides.LEFT);
        assertEquals(expected, coralQueue.getNext());

        expected =
                new CoralQueue.CoralPosition(
                        "12L4",
                        ScoringConstants.CORAL_POSITIONS[23],
                        ScoringConstants.ScoringHeights.L4,
                        ScoringConstants.ScoringSides.LEFT);
        assertEquals(expected, coralQueue.getNext());

        expected =
                new CoralQueue.CoralPosition(
                        "9L4",
                        ScoringConstants.CORAL_POSITIONS[20],
                        ScoringConstants.ScoringHeights.L4,
                        ScoringConstants.ScoringSides.RIGHT);

        assertEquals(expected, coralQueue.getNext());

        DriverStationSim.setAllianceStationId(AllianceStationID.Red1); // Set back to red
        DriverStationSim.notifyNewData();
    }
}
