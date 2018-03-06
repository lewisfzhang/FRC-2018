package com.team254.lib.physics;

import com.team254.lib.util.Util;

/**
 * Model of a DC motor rotating a shaft.  All parameters refer to the output (e.g. should already consider gearing
 * and efficiency losses).  The motor is assumed to be symmetric forward/reverse.
 */
public class DCMotorTransmission {
    // TODO add electrical constants?  (e.g. current)

    // All units must be SI!
    protected final double speed_per_volt_straight_;  // rad/s per V (no load)protected final double speed_per_volt_straight_;  // rad/s per V (no load)
    protected final double speed_per_volt_turn_in_place_;  // rad/s per V (no load)protected final double speed_per_volt_straight_;  // rad/s per V (no load)
    protected final double torque_per_volt_straight_;  // N m per V (stall)
    protected final double torque_per_volt_turn_in_place_;  // N m per V (stall)
    protected final double friction_voltage_straight_;  // V
    protected final double friction_voltage_turn_in_place_;  // V

    private static double mix(double straight_value, double turn_in_place_value, double curvature) {
        double rv = straight_value + turn_in_place_value * Math.abs(curvature);
        if (Double.isInfinite(rv)) {
            return turn_in_place_value;
        } else {
            return rv / Math.sqrt(curvature * curvature + 1);
        }
    }

    public DCMotorTransmission(final double speed_per_volt_straight,
                               final double speed_per_volt_turn_in_place,
                               final double torque_per_volt_straight,
                               final double torque_per_volt_turn_in_place,
                               final double friction_voltage_straight,
                               final double friction_voltage_turn_in_place) {
        speed_per_volt_straight_ = speed_per_volt_straight;
        speed_per_volt_turn_in_place_ = speed_per_volt_turn_in_place;
        torque_per_volt_straight_ = torque_per_volt_straight;
        torque_per_volt_turn_in_place_ = torque_per_volt_turn_in_place;
        friction_voltage_straight_ = friction_voltage_turn_in_place;
        friction_voltage_turn_in_place_ = friction_voltage_turn_in_place;
    }

    public double speed_per_volt_straight() {
        return speed_per_volt_straight_;
    }
    public double speed_per_volt_turn_in_place() {
        return speed_per_volt_turn_in_place_;
    }
    public double speed_per_volt(double curvature) {
        return mix(speed_per_volt_straight_, speed_per_volt_turn_in_place_, curvature);
    }
    public double torque_per_volt_straight_() {
        return torque_per_volt_straight_;
    }
    public double torque_per_volt_turn_in_place() {
        return torque_per_volt_turn_in_place_;
    }
    public double torque_per_volt(double curvature) {
        return mix(torque_per_volt_straight_, torque_per_volt_turn_in_place_, curvature);
    }
    public double friction_voltage_straight() {
        return friction_voltage_straight_;
    }
    public double friction_voltage_turn_in_place() {
        return friction_voltage_turn_in_place_;
    }
    public double friction_voltage(double curvature) {
        return mix(friction_voltage_straight_, friction_voltage_turn_in_place_, curvature);
    }

    public double free_speed_at_voltage(final double curvature, final double voltage) {
        if (voltage > Util.kEpsilon) {
            return Math.max(0.0, voltage - friction_voltage(curvature)) * speed_per_volt(curvature);
        } else if (voltage < Util.kEpsilon) {
            return Math.min(0.0, voltage + friction_voltage(curvature)) * speed_per_volt(curvature);
        } else {
            return 0.0;
        }
    }

    public double getTorqueForVoltage(final double curvature, final double output_speed, final double voltage) {
        double effective_voltage = voltage;
        if (output_speed > Util.kEpsilon) {
            // Forward motion, rolling friction.
            effective_voltage -= friction_voltage(curvature);
        } else if (output_speed < -Util.kEpsilon) {
            // Reverse motion, rolling friction.
            effective_voltage += friction_voltage(curvature);
        } else if (voltage > Util.kEpsilon) {
            // System is static, forward torque.
            effective_voltage = Math.max(0.0, voltage - friction_voltage(curvature));
        } else if (voltage < -Util.kEpsilon) {
            // System is static, reverse torque.
            effective_voltage = Math.min(0.0, voltage + friction_voltage(curvature));
        } else {
            // System is idle.
            return 0.0;
        }
        return torque_per_volt(curvature) * (-output_speed / speed_per_volt(curvature) + effective_voltage);
    }

    public double getVoltageForTorque(final double curvature, final double output_speed, final double torque) {
        double friction_voltage;
        if (output_speed > Util.kEpsilon) {
            // Forward motion, rolling friction.
            friction_voltage = friction_voltage(curvature);
        } else if (output_speed < -Util.kEpsilon) {
            // Reverse motion, rolling friction.
            friction_voltage = -friction_voltage(curvature);
        } else if (torque > Util.kEpsilon) {
            // System is static, forward torque.
            friction_voltage = friction_voltage(curvature);
        } else if (torque < -Util.kEpsilon) {
            // System is static, reverse torque.
            friction_voltage = -friction_voltage(curvature);
        } else {
            // System is idle.
            return 0.0;
        }
        return torque / torque_per_volt(curvature) + output_speed / speed_per_volt(curvature) + friction_voltage;
    }
}
