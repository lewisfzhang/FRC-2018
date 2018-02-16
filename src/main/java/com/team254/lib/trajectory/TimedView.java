package com.team254.lib.trajectory;

import com.team254.lib.geometry.State;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.Util;

public class TimedView<S extends State<S>> implements TrajectoryView<TimedState<S>> {
    protected Trajectory<TimedState<S>> trajectory_;
    protected double start_t_;
    protected double end_t_;

    public TimedView(Trajectory<TimedState<S>> trajectory) {
        trajectory_ = trajectory;
        start_t_ = trajectory_.getState(0).t();
        end_t_ = trajectory_.getState(trajectory_.length() - 1).t();
    }

    @Override
    public double first_interpolant() {
        return start_t_;
    }

    @Override
    public double last_interpolant() {
        return end_t_;
    }

    @Override
    public TrajectorySamplePoint<TimedState<S>> sample(double interpolant) {
        if (interpolant >= end_t_) {
            return new TrajectorySamplePoint<TimedState<S>>(trajectory_.getPoint(trajectory_.length() - 1));
        }
        if (interpolant <= start_t_) {
            return new TrajectorySamplePoint<TimedState<S>>(trajectory_.getPoint(0));
        }
        for (int i = 1; i < trajectory_.length(); ++i) {
            final TrajectoryPoint<TimedState<S>> s = trajectory_.getPoint(i);
            if (s.state().t() >= interpolant) {
                final TrajectoryPoint<TimedState<S>> prev_s = trajectory_.getPoint(i - 1);
                if (Util.epsilonEquals(s.state().t(), prev_s.state().t()) || prev_s.state().t() > s.state().t()) {
                    return new TrajectorySamplePoint<TimedState<S>>(s);
                }
                return new TrajectorySamplePoint<TimedState<S>>(prev_s.state().interpolate(s.state(),
                        (interpolant - prev_s.state().t()) / (s.state().t() - prev_s.state().t())), i - 1, i);
            }
        }
        throw new RuntimeException();
    }
}
