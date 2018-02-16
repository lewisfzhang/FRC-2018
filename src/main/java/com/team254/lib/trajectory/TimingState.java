package com.team254.lib.trajectory;

import java.text.DecimalFormat;

public class TimingState {
    protected double t_; // Time we achieve this state.
    protected double velocity_; // ds/dt
    protected double acceleration_; // d^2s/dt^2

    public TimingState(double t, double velocity, double acceleration) {
        set_t(t);
        set_velocity(velocity);
        set_acceleration(acceleration);
    }

    public void set_t(double t) {
        t_ = t;
    }

    public double t() {
        return t_;
    }

    public void set_velocity(double velocity) {
        velocity_ = velocity;
    }

    public double velocity() {
        return velocity_;
    }

    public void set_acceleration(double acceleration) {
        acceleration_ = acceleration;
    }

    public double acceleration() {
        return acceleration_;
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "t: " + fmt.format(t()) + ", v: " + fmt.format(velocity()) + ", a: " + fmt.format(acceleration());
    }

    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(t()) + "," + fmt.format(velocity()) + "," + fmt.format(acceleration());
    }
}
