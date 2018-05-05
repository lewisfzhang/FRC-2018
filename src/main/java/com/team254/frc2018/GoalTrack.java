package com.team254.frc2018;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.Map;
import java.util.TreeMap;

/**
 * A class that is used to keep track of all goals detected by the vision system. As goals are detected/not detected
 * anymore by the vision system, function calls will be made to create, destroy, or update a goal track.
 *
 * This helps in the goal ranking process that determines which goal to fire into, and helps to smooth measurements of
 * the goal's location over time.
 *
 */
public class GoalTrack {
    private static final double kMaxTrackerDistance = 12.0;
    private static final double kMaxGoalTrackAge = 1.0;
    private static final double kCameraFrameRate = 90.0;

    Map<Double, Pose2d> mObservedPositions = new TreeMap<>();
    Pose2d mSmoothedPosition = null;
    int mId;

    private GoalTrack() {
    }

    /**
     * Makes a new track based on the timestamp and the goal's coordinates (from vision)
     */
    public static GoalTrack makeNewTrack(double timestamp, Pose2d first_observation, int id) {
        GoalTrack rv = new GoalTrack();
        rv.mObservedPositions.put(timestamp, first_observation);
        rv.mSmoothedPosition = first_observation;
        rv.mId = id;
        return rv;
    }

    public void emptyUpdate() {
        pruneByTime();
    }

    /**
     * Attempts to update the track with a new observation.
     *
     * @return True if the track was updated
     */
    public boolean tryUpdate(double timestamp, Pose2d new_observation) {
        if (!isAlive()) {
            return false;
        }
        double distance = new Translation2d(mSmoothedPosition.getTranslation(), new_observation.getTranslation()).norm();
        if (distance < kMaxTrackerDistance) {
            mObservedPositions.put(timestamp, new_observation);
            pruneByTime();
            return true;
        } else {
            emptyUpdate();
            return false;
        }
    }

    public boolean isAlive() {
        return mObservedPositions.size() > 0;
    }

    /**
     * Removes the track if it is older than the set "age" described in the Constants file.
     *
     */
    void pruneByTime() {
        double delete_before = Timer.getFPGATimestamp() - kMaxGoalTrackAge;

        mObservedPositions.entrySet().removeIf((Map.Entry<Double, Pose2d> position) -> position.getKey() < delete_before);

        if (mObservedPositions.isEmpty()) {
            mSmoothedPosition = null;
        } else {
            smooth();
        }
    }

    /**
     * Averages out the observed positions based on an set of observed positions
     */
    void smooth() {
        if (isAlive()) {
            Translation2d sumPosition = new Translation2d();
            Translation2d sumRotation = new Translation2d();
            for (Map.Entry<Double, Pose2d> entry : mObservedPositions.entrySet()) {
                sumPosition = sumPosition.translateBy(entry.getValue().getTranslation());
                sumRotation = sumRotation.translateBy(entry.getValue().getRotation().toTranslation());

            }
            mSmoothedPosition = new Pose2d(sumPosition.scale(1.0 / mObservedPositions.size()),
                    new Rotation2d(sumRotation.scale(1.0 / mObservedPositions.size()), true));
        }
    }

    public Pose2d getSmoothedPosition() {
        return mSmoothedPosition;
    }

    public double getLatestTimestamp() {
        return mObservedPositions.keySet().stream().max(Double::compareTo).orElse(0.0);
    }

    public double getStability() {
        return Math.min(1.0, mObservedPositions.size() / (kCameraFrameRate * kMaxGoalTrackAge));
    }

    public int getId() {
        return mId;
    }
}
