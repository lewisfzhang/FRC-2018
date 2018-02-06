package com.team254.lib.trajectory.timing;

import com.team254.lib.geometry.State;
import com.team254.lib.trajectory.DistanceView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.util.Util;

import java.util.ArrayList;
import java.util.List;

public class TimingUtil {
    protected static class ConstrainedState<S extends State<S>> {
        public S state;
        public double distance;
        public double max_velocity;
        public double min_acceleration;
        public double max_acceleration;
    }

    public static <S extends State<S>> Trajectory<TimedState<S>> timeParameterizeTrajectory(
            final DistanceView<S> distance_view,
            double step_size,
            final List<TimingConstraint<S>> constraints,
            double start_velocity,
            double end_velocity,
            double max_velocity,
            double max_abs_acceleration) {
        final int num_states = (int) Math.ceil(distance_view.last_interpolant() / step_size);
        List<S> states = new ArrayList<>(num_states);
        for (int i = 0; i < num_states; ++i) {
            states.add(distance_view.sample(Math.min(i * step_size, distance_view.last_interpolant())).state());
        }
        return timeParameterizeTrajectory(states, constraints, start_velocity, end_velocity,
                max_velocity, max_abs_acceleration);
    }

    public static <S extends State<S>> Trajectory<TimedState<S>> timeParameterizeTrajectory(
            final List<S> states,
            final List<TimingConstraint<S>> constraints,
            double start_velocity,
            double end_velocity,
            double max_velocity,
            double max_abs_acceleration) {
        List<ConstrainedState<S>> constraint_states = new ArrayList<>(states.size());

        // Forward pass. We look at pairs of consecutive states, where the start state has already been velocity
        // parameterized (though we may adjust the velocity downwards during the backwards pass). We wish to find an
        // acceleration that is admissible at both the start and end state, as well as an admissible end velocity. If
        // there is no admissible end velocity or acceleration, we set the end velocity to the state's maximum allowed
        // velocity and will repair the acceleration during the backward pass (by slowing down the predecessor).
        ConstrainedState<S> predecessor = new ConstrainedState<>();
        predecessor.state = states.get(0);
        predecessor.distance = 0.0;
        predecessor.max_velocity = start_velocity;
        predecessor.min_acceleration = -max_abs_acceleration;
        predecessor.max_acceleration = max_abs_acceleration;
        for (int i = 0; i < states.size(); ++i) {
            // Add the new state.
            constraint_states.add(new ConstrainedState<>());
            ConstrainedState<S> constraint_state = constraint_states.get(i);
            constraint_state.state = states.get(i);
            final double ds = constraint_state.state.distance(predecessor.state);
            constraint_state.distance = ds + predecessor.distance;

            // We may need to iterate to find the maximum end velocity and common acceleration, since acceleration
            // limits may be a function of velocity.
            while (true) {
                // Enforce global max velocity and max reachable velocity by global acceleration limit.
                // vf = sqrt(vi^2 + 2*a*d)
                constraint_state.max_velocity = Math.min(max_velocity,
                        Math.sqrt(predecessor.max_velocity * predecessor.max_velocity
                                + 2.0 * predecessor.max_acceleration * ds));
                if (Double.isNaN(constraint_state.max_velocity)) {
                    throw new RuntimeException();
                }
                // Enforce global max absolute acceleration.
                constraint_state.min_acceleration = -max_abs_acceleration;
                constraint_state.max_acceleration = max_abs_acceleration;

                // At this point, the state is full constructed, but no constraints have been applied aside from
                // predecessor
                // state max accel.

                // Enforce all velocity constraints.
                for (final TimingConstraint<S> constraint : constraints) {
                    constraint_state.max_velocity = Math.min(constraint_state.max_velocity,
                            constraint.getMaxVelocity(constraint_state.state));
                }
                if (constraint_state.max_velocity < 0.0) {
                    // This should never happen if constraints are well-behaved.
                    throw new RuntimeException();
                }

                // Now enforce all acceleration constraints.
                for (final TimingConstraint<S> constraint : constraints) {
                    final TimingConstraint.MinMaxAcceleration min_max_accel = constraint.getMinMaxAcceleration(
                            constraint_state.state,
                            constraint_state.max_velocity);
                    if (!min_max_accel.valid()) {
                        // This should never happen if constraints are well-behaved.
                        throw new RuntimeException();
                    }
                    constraint_state.min_acceleration = Math.max(constraint_state.min_acceleration,
                            min_max_accel.min_acceleration());
                    constraint_state.max_acceleration = Math.min(constraint_state.max_acceleration,
                            min_max_accel.max_acceleration());
                }
                if (constraint_state.min_acceleration > constraint_state.max_acceleration) {
                    // This should never happen if constraints are well-behaved.
                    throw new RuntimeException();
                }

                if (ds < Util.kEpsilon) {
                    break;
                }
                // If the max acceleration for this constraint state is more conservative than what we have applied, we
                // need to reduce the max accel and try again.
                // TODO: Simply using the new max acceleration is guaranteed to be valid, but may be too conservative.
                // Doing a search would be better.
                final double actual_acceleration = (constraint_state.max_velocity * constraint_state.max_velocity
                        - predecessor.max_velocity * predecessor.max_velocity) / (2.0 * ds);
                if (constraint_state.max_acceleration < actual_acceleration - Util.kEpsilon) {
                    predecessor.max_acceleration = constraint_state.max_acceleration;
                } else {
                    if (actual_acceleration > predecessor.min_acceleration + Util.kEpsilon) {
                        predecessor.max_acceleration = actual_acceleration;
                    }
                    break;
                }
            }
            predecessor = constraint_state;
        }

        // Backward pass.
        ConstrainedState<S> successor = new ConstrainedState<>();
        successor.state = states.get(states.size() - 1);
        successor.distance = constraint_states.get(states.size() - 1).distance;
        successor.max_velocity = end_velocity;
        successor.min_acceleration = -max_abs_acceleration;
        successor.max_acceleration = max_abs_acceleration;
        for (int i = states.size() - 1; i >= 0; --i) {
            ConstrainedState<S> constraint_state = constraint_states.get(i);
            final double ds = constraint_state.distance - successor.distance; // will be negative.

            while (true) {
                // Enforce reverse max reachable velocity limit.
                // vf = sqrt(vi^2 + 2*a*d), where vi = successor.
                final double new_max_velocity = Math.sqrt(successor.max_velocity * successor.max_velocity
                        + 2.0 * successor.min_acceleration * ds);
                if (new_max_velocity >= constraint_state.max_velocity) {
                    // No new limits.
                    break;
                }
                constraint_state.max_velocity = new_max_velocity;
                if (Double.isNaN(constraint_state.max_velocity)) {
                    throw new RuntimeException();
                }

                // Now enforce all acceleration constraints on the lower max velocity.
                for (final TimingConstraint<S> constraint : constraints) {
                    final TimingConstraint.MinMaxAcceleration min_max_accel = constraint.getMinMaxAcceleration(
                            constraint_state.state,
                            constraint_state.max_velocity);
                    if (!min_max_accel.valid()) {
                        throw new RuntimeException();
                    }
                    constraint_state.min_acceleration = Math.max(constraint_state.min_acceleration,
                            min_max_accel.min_acceleration());
                    constraint_state.max_acceleration = Math.min(constraint_state.max_acceleration,
                            min_max_accel.max_acceleration());
                }
                if (constraint_state.min_acceleration > constraint_state.max_acceleration) {
                    throw new RuntimeException();
                }

                if (ds > Util.kEpsilon) {
                    break;
                }
                // If the min acceleration for this constraint state is more conservative than what we have applied, we
                // need to reduce the max accel and try again.
                // TODO: Simply using the new max acceleration is guaranteed to be valid, but may be too conservative.
                // Doing a search would be better.
                final double actual_acceleration = (constraint_state.max_velocity * constraint_state.max_velocity
                        - successor.max_velocity * successor.max_velocity) / (2.0 * ds);
                if (constraint_state.min_acceleration > actual_acceleration + Util.kEpsilon) {
                    successor.min_acceleration = successor.max_acceleration = constraint_state.min_acceleration;
                } else {
                    successor.min_acceleration = successor.max_acceleration = actual_acceleration;
                    break;
                }
            }
            successor = constraint_state;
        }

        // Integrate the constrained states forward in time to obtain the TimedStates.
        List<TimedState<S>> timed_states = new ArrayList<>(states.size());
        double t = 0.0;
        double s = 0.0;
        double v = 0.0;
        double min_a = 0.0;
        double max_a = 0.0;
        for (int i = 0; i < states.size(); ++i) {
            final ConstrainedState<S> constrained_state = constraint_states.get(i);
            // Advance t.
            final double dv = constrained_state.max_velocity - v;
            if (i > 0) {
                if (dv < -Util.kEpsilon) {
                    // We are decelerating.
                    t += dv / min_a;
                    timed_states.get(i - 1).set_acceleration(min_a);
                } else if (dv > Util.kEpsilon) {
                    // We are accelerating.
                    t += dv / max_a;
                    timed_states.get(i - 1).set_acceleration(max_a);
                } else if (constrained_state.max_velocity > Util.kEpsilon) {
                    // Constant velocity.
                    t += (constrained_state.distance - s) / constrained_state.max_velocity;
                    timed_states.get(i - 1).set_acceleration(0.0);
                }
            }
            if (Double.isNaN(t) || Double.isInfinite(t)) {
                throw new RuntimeException();
            }

            v = constrained_state.max_velocity;
            s = constrained_state.distance;
            max_a = constrained_state.max_acceleration;
            min_a = constrained_state.min_acceleration;
            timed_states.add(new TimedState<S>(constrained_state.state, t, v, max_a));
        }
        return new Trajectory<>(timed_states);
    }
}