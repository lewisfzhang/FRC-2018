package com.team254.frc2018;

import com.team254.lib.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;

/**
 * This is used in the event that multiple goals are detected to judge all goals based on timestamp, stability, and
 * continuation of previous goals (i.e. if a goal was detected earlier and has changed locations). This allows the robot
 * to make consistent decisions about which goal to aim at and to smooth out jitter from vibration of the camera.
 */
public class GoalTracker {
    /**
     * Track reports contain all of the relevant information about a given goal track.
     */
    public static class TrackReport {
        // Translation from the field frame to the goal
        public Pose2d field_to_goal;

        // The timestamp of the latest time that the goal has been observed
        public double latest_timestamp;

        // The percentage of the goal tracking time during which this goal has
        // been observed (0 to 1)
        public double stability;

        // The track id
        public int id;

        public TrackReport(GoalTrack track) {
            this.field_to_goal = track.getSmoothedPosition();
            this.latest_timestamp = track.getLatestTimestamp();
            this.stability = track.getStability();
            this.id = track.getId();
        }
    }

    List<GoalTrack> mCurrentTracks = new ArrayList<>();
    int mNextId = 0;

    public GoalTracker() {
    }

    public void reset() {
        mCurrentTracks.clear();
    }

    public void update(double timestamp, Pose2d target) {
        // Try to update existing tracks
        boolean hasUpdatedTrack = false;
        for (GoalTrack track : mCurrentTracks) {
            if (!hasUpdatedTrack) {
                if (track.tryUpdate(timestamp, target)) {
                    hasUpdatedTrack = true;
                }
            } else {
                track.emptyUpdate();
            }
        }
        // Prune any tracks that have died
        mCurrentTracks.removeIf((GoalTrack track) -> !track.isAlive());

        // If all tracks are dead, start new tracks for any detections
        if (mCurrentTracks.isEmpty()) {
            mCurrentTracks.add(GoalTrack.makeNewTrack(timestamp, target, mNextId));
            ++mNextId;
        }
    }

    public boolean hasTracks() {
        return !mCurrentTracks.isEmpty();
    }

    public List<TrackReport> getTracks() {
        List<TrackReport> rv = new ArrayList<>();
        for (GoalTrack track : mCurrentTracks) {
            rv.add(new TrackReport(track));
        }
        return rv;
    }
}