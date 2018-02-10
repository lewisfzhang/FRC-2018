package com.team254.lib.physics;

import com.team254.lib.util.Util;

public class DCMotorTransmission {
    // TODO add electrical constants?  (e.g. current)

    // All units must be SI!
    protected final double gear_reduction_;  // ratio
    protected final double efficiency_;  // unitless, [0, 1]
    protected final double stall_torque_;  // N m
    protected final double free_speed_;  // rad/s
    protected final double nominal_voltage_;  // V
    protected final double friction_torque_;  // N m (assume identical for both static and rolling friction)
    protected final double torque_constant_;  // N m

    // Free speed is actual observed speed at nominal voltage (e.g. it should account for speed loss due to friction).
    public DCMotorTransmission(final double gear_reduction, final double efficiency, final double stall_torque, final double free_speed, final double nominal_voltage, final double friction_torque) {
        gear_reduction_ = gear_reduction;
        efficiency_ = efficiency;
        stall_torque_ = stall_torque;
        free_speed_ = free_speed;
        nominal_voltage_ = nominal_voltage;
        friction_torque_ = friction_torque;

        torque_constant_ = gear_reduction_ * efficiency_ * stall_torque_;
    }

    // TODO add more useful constructors

    public double gear_reduction() {
        return gear_reduction_;
    }

    public double efficiency() {
        return efficiency_;
    }

    public double stall_torque() {
        return stall_torque_;
    }

    public double free_speed() {
        return free_speed_;
    }

    public double nominal_voltage() {
        return nominal_voltage_;
    }

    public double friction_torque() {
        return friction_torque_;
    }

    public double torque_constant() {
        return torque_constant_;
    }

    // TODO interpret free speed/stall torque as actual observed values (compensate for friction).
    public double getTorqueForVoltage(double output_speed, double voltage) {
        final double frictionless_torque = torque_constant_ * (-output_speed * gear_reduction_ / free_speed_ + voltage / nominal_voltage_);
        if (Util.epsilonEquals(Math.abs(output_speed), 0.0)) {
            // Static friction opposes force/torque.
            if (frictionless_torque >= 0.0) {
                return Math.min(0.0, frictionless_torque - friction_torque_);
            } else {
                return Math.max(0.0, frictionless_torque + friction_torque_);
            }
        }
        // Dynamic friction opposes motion.
        if (output_speed > 0.0) {
            return frictionless_torque - friction_torque_;
        } else {
            return frictionless_torque + friction_torque_;
        }
    }

    public double getVoltageForTorque(double output_speed, double torque) {
        if (Util.epsilonEquals(Math.abs(torque), 0.0)) {
            return 0.0;
        }
        if (output_speed > 0.0) {
            return ((torque + friction_torque_) / torque_constant_ + output_speed * gear_reduction_ / free_speed_) * nominal_voltage_;
        } else {
            return ((torque - friction_torque_) / torque_constant_ + output_speed * gear_reduction_ / free_speed_) * nominal_voltage_;
        }
    }
}
