package com.team254.lib.physics;

import com.team254.lib.util.CSVWritable;
import com.team254.lib.util.Util;

import java.text.DecimalFormat;
import java.util.Arrays;

/**
 * Dynamic model a differential drive robot.  Note: to simplify things, this math assumes the center of mass is
 * coincident with the kinematic center of rotation (e.g. midpoint of the center axle).
 */
public class DifferentialDrive {
    // All units must be SI!

    // TODO
    // deal with the case that one side is moving but the other side is stationary and has not broken static
    // friction (so the whole robot pivots about the stationary side).  Equation for this looks like:
    // F_l + F_r = linear_load*a
    // F_r - F_l = angular_load*alpha
    // s = Sgn(a/alpha)
    // s * effective_wheelbase_width = a/alpha
    // ~
    // F_r * (s * linear_load * effective_wheelbase_width - angular_load) == F_l * (s * linear_load *
    // effective_wheelbase_width + angular_load)
    // Need to then check if the force at the stationary side breaks static friction.
    // This is not a big deal for the main use-cases of this class (designing feasible trajectories and computing
    // feedforward voltages).

    // Equivalent mass when accelerating purely linearly, in kg.
    // This is "equivalent" in that it also absorbs the effects of drivetrain inertia.
    // Theoretical value:
    // = m_robot + I_transmissions / r_wheel^2
    protected final double linear_inertia_;

    // Equivalent load when accelerating purely angularly, in kg*m.
    // Normally a moment of inertia is in kg*m^2, but in this case we always know the radius at which torques are
    // applied so can cancel that term.
    // Theoretical value:
    // = I_robot_about_com / wheelbase_radius + I_transmissions / r_wheel
    protected final double angular_inertia_;

    protected final double wheel_radius_;  // m

    // "Effective" kinematic wheelbase radius.  Might be larger than theoretical to compensate for skid steer.
    protected final double effective_wheelbase_radius_;  // m
    protected final DCMotorTransmission left_transmission_;
    protected final DCMotorTransmission right_transmission_;

    public DifferentialDrive(final double linear_inertia,
                             final double angular_inertia,
                             final double wheel_radius,
                             final double effective_wheelbase_radius,
                             final DCMotorTransmission left_transmission,
                             final DCMotorTransmission right_transmission) {
        linear_inertia_ = linear_inertia;
        angular_inertia_ = angular_inertia;
        wheel_radius_ = wheel_radius;
        effective_wheelbase_radius_ = effective_wheelbase_radius;
        left_transmission_ = left_transmission;
        right_transmission_ = right_transmission;
    }

    public double linear_inertia() {
        return linear_inertia_;
    }

    public double angular_inertia() {
        return angular_inertia_;
    }

    public double wheel_radius() {
        return wheel_radius_;
    }

    public double effective_wheelbase_radius() {
        return effective_wheelbase_radius_;
    }

    public DCMotorTransmission left_transmission() {
        return left_transmission_;
    }

    public DCMotorTransmission right_transmission() {
        return right_transmission_;
    }

    // Input/demand could be either velocity or acceleration...the math is the same.
    public ChassisState solveForwardKinematics(final WheelState wheel_motion) {
        ChassisState chassis_motion = new ChassisState();
        chassis_motion.linear = wheel_radius_ * (wheel_motion.right + wheel_motion.left) / 2.0;
        chassis_motion.angular = wheel_radius_ * (wheel_motion.right - wheel_motion.left) / (2.0 *
                effective_wheelbase_radius_);
        return chassis_motion;
    }

    // Input/output could be either velocity or acceleration...the math is the same.
    public WheelState solveInverseKinematics(final ChassisState chassis_motion) {
        WheelState wheel_motion = new WheelState();
        wheel_motion.left = (chassis_motion.linear - effective_wheelbase_radius_ * chassis_motion.angular) /
                wheel_radius_;
        wheel_motion.right = (chassis_motion.linear + effective_wheelbase_radius_ * chassis_motion.angular) /
                wheel_radius_;
        return wheel_motion;
    }

    // Solve for torques and accelerations.
    public DriveDynamics solveForwardDynamics(final ChassisState chassis_velocity, final WheelState voltage) {
        DriveDynamics dynamics = new DriveDynamics();
        dynamics.wheel_velocity = solveInverseKinematics(chassis_velocity);
        dynamics.chassis_velocity = chassis_velocity;
        dynamics.voltage = voltage;
        solveForwardDynamics(dynamics);
        return dynamics;
    }

    public DriveDynamics solveForwardDynamics(final WheelState wheel_velocity, final WheelState voltage) {
        DriveDynamics dynamics = new DriveDynamics();
        dynamics.wheel_velocity = wheel_velocity;
        dynamics.chassis_velocity = solveForwardKinematics(wheel_velocity);
        dynamics.voltage = voltage;
        solveForwardDynamics(dynamics);
        return dynamics;
    }

    // Assumptions about dynamics: velocities and voltages provided.
    public void solveForwardDynamics(DriveDynamics dynamics) {
        // TODO this assumes when stationary we are always accelerating with 0 curvature.
        final boolean left_stationary = Util.epsilonEquals(dynamics.wheel_velocity.left, 0.0) && Math.abs(dynamics
                .voltage.left) < left_transmission_.friction_voltage();
        final boolean right_stationary = Util.epsilonEquals(dynamics.wheel_velocity.right, 0.0) && Math.abs(dynamics
                .voltage.right) < right_transmission_.friction_voltage();
        if (left_stationary && right_stationary) {
            // Neither side breaks static friction.
            dynamics.wheel_torque.left = dynamics.wheel_torque.right = 0.0;
            dynamics.chassis_acceleration.linear = dynamics.chassis_acceleration.angular = 0.0;
            dynamics.wheel_acceleration.left = dynamics.wheel_acceleration.right = 0.0;
            return;
        }
        double curvature = dynamics.chassis_velocity.angular / dynamics.chassis_velocity.linear;
        if (Double.isNaN(curvature)) curvature = 0.0;

        // Solve for motor torques generated on each side.
        dynamics.wheel_torque.left = left_transmission_.getTorqueForVoltage(dynamics.wheel_velocity.left, dynamics
                .voltage.left);
        dynamics.wheel_torque.right = right_transmission_.getTorqueForVoltage(dynamics.wheel_velocity.right, dynamics
                .voltage.right);

        // Add forces and torques about the center of mass.
        dynamics.chassis_acceleration.linear = (dynamics.wheel_torque.right + dynamics.wheel_torque.left) /
                (wheel_radius_ * linear_inertia_);
        dynamics.chassis_acceleration.angular = (dynamics.wheel_torque.right - dynamics.wheel_torque.left) /
                (wheel_radius_ * angular_inertia_);

        // Resolve chassis accelerations to each wheel.
        dynamics.wheel_acceleration.left = dynamics.chassis_acceleration.linear - dynamics.chassis_acceleration
                .angular * effective_wheelbase_radius_;
        dynamics.wheel_acceleration.right = dynamics.chassis_acceleration.linear + dynamics.chassis_acceleration
                .angular * effective_wheelbase_radius_;
    }

    // Solve for torques and voltages.
    public DriveDynamics solveInverseDynamics(final ChassisState chassis_velocity, final ChassisState
            chassis_acceleration) {
        DriveDynamics dynamics = new DriveDynamics();
        dynamics.chassis_velocity = chassis_velocity;
        dynamics.chassis_acceleration = chassis_acceleration;
        dynamics.wheel_velocity = solveInverseKinematics(chassis_velocity);
        dynamics.wheel_acceleration = solveInverseKinematics(chassis_acceleration);
        solveInverseDynamics(dynamics);
        return dynamics;
    }

    public DriveDynamics solveInverseDynamics(final WheelState wheel_velocity, final WheelState wheel_acceleration) {
        DriveDynamics dynamics = new DriveDynamics();
        dynamics.chassis_velocity = solveForwardKinematics(wheel_velocity);
        dynamics.chassis_acceleration = solveForwardKinematics(wheel_acceleration);
        dynamics.wheel_velocity = wheel_velocity;
        dynamics.wheel_acceleration = wheel_acceleration;
        solveInverseDynamics(dynamics);
        return dynamics;
    }

    // Assumptions about dynamics: velocities and accelerations provided.
    public void solveInverseDynamics(DriveDynamics dynamics) {
        // Determine the necessary torques on the left and right wheels to produce the desired wheel accelerations.
        dynamics.wheel_torque.left = wheel_radius_ / 2.0 * (dynamics.chassis_acceleration.linear * linear_inertia_ -
                dynamics.chassis_acceleration.angular * angular_inertia_);
        dynamics.wheel_torque.right = wheel_radius_ / 2.0 * (dynamics.chassis_acceleration.linear * linear_inertia_ +
                dynamics.chassis_acceleration.angular * angular_inertia_);

        double curvature = dynamics.chassis_velocity.angular / dynamics.chassis_velocity.linear;
        if (Double.isNaN(curvature)) curvature = 0.0;
        // Solve for input voltages.
        dynamics.voltage.left = left_transmission_.getVoltageForTorque(dynamics.wheel_velocity.left, dynamics
                .wheel_torque.left);
        dynamics.voltage.right = right_transmission_.getVoltageForTorque(dynamics.wheel_velocity.right, dynamics
                .wheel_torque.right);
    }

    public double getMaxAbsVelocity(double curvature, double max_abs_voltage) {
        // k = w / v
        // v = r_w*(wr + wl) / 2
        // w = r_w*(wr - wl) / (2 * r_wb)
        // Plug in max_abs_voltage for each wheel.
        final double left_speed_at_max_voltage = left_transmission_.free_speed_at_voltage(max_abs_voltage);
        final double right_speed_at_max_voltage = right_transmission_.free_speed_at_voltage(max_abs_voltage);
        if (Util.epsilonEquals(curvature, 0.0)) {
            return wheel_radius_ * Math.min(left_speed_at_max_voltage, right_speed_at_max_voltage);
        }
        if (Double.isInfinite(curvature)) {
            // Turn in place.  Return value meaning becomes angular velocity.
            final double wheel_speed = Math.min(left_speed_at_max_voltage, right_speed_at_max_voltage);
            return Math.signum(curvature) * wheel_radius_ * wheel_speed / effective_wheelbase_radius_;
        }

        final double right_speed_if_left_max = left_speed_at_max_voltage * (effective_wheelbase_radius_ * curvature +
                1.0) / (1.0 - effective_wheelbase_radius_ * curvature);
        if (Math.abs(right_speed_if_left_max) <= right_speed_at_max_voltage + Util.kEpsilon) {
            return wheel_radius_ * (left_speed_at_max_voltage + right_speed_if_left_max) / 2.0;
        }
        final double left_speed_if_right_max = right_speed_at_max_voltage * (1.0 - effective_wheelbase_radius_ *
                curvature) / (1.0 + effective_wheelbase_radius_ * curvature);
        // assert Math.abs(left_speed_if_right_max) <= left_speed_at_max_voltage + Util.kEpsilon;
        return wheel_radius_ * (right_speed_at_max_voltage + left_speed_if_right_max) / 2.0;
    }

    public static class MinMax {
        public double min;
        public double max;
    }

    // NOTE: curvature_hint is redundant here in the case that chassis_velocity is non-stationary.  It is only read if the
    // robot is stationary.
    public MinMax getMinMaxAcceleration(final ChassisState chassis_velocity, double curvature_hint, double
            max_abs_voltage) {
        MinMax result = new MinMax();
        double curvature = chassis_velocity.angular / chassis_velocity.linear;
        if (Double.isNaN(curvature)) {
            curvature = curvature_hint;
        }
        final WheelState wheel_velocities = solveInverseKinematics(chassis_velocity);
        result.min = Double.POSITIVE_INFINITY;
        result.max = Double.NEGATIVE_INFINITY;

        // Math:
        // r_w * (Tl + Tr) = m*a
        // r_w * r_wb * (Tr - Tl) = i*alpha
        // k = alpha/a

        // 2 equations, 2 unknowns.
        // Solve for a and (Tl|Tr)

        final double linear_term = Double.isInfinite(curvature) ? 0.0 : linear_inertia_ * effective_wheelbase_radius_;
        final double angular_term = Double.isInfinite(curvature) ? angular_inertia_ : angular_inertia_ * curvature;

        // Check all four cases and record the min and max valid accelerations.
        for (boolean left : Arrays.asList(false, true)) {
            for (double sign : Arrays.asList(1.0, -1.0)) {
                final DCMotorTransmission fixed_transmission = left ? left_transmission_ : right_transmission_;
                final DCMotorTransmission variable_transmission = left ? right_transmission_ : left_transmission_;
                final double fixed_torque = fixed_transmission.getTorqueForVoltage(wheel_velocities.get(left), sign *
                        max_abs_voltage);
                double variable_torque = 0.0;
                if (left) {
                    variable_torque = fixed_torque * (linear_term + angular_term) / (linear_term - angular_term);
                } else {
                    variable_torque = fixed_torque * (linear_term - angular_term) / (linear_term + angular_term);
                }
                final double variable_voltage = variable_transmission.getVoltageForTorque(wheel_velocities.get(!left), variable_torque);
                if (Math.abs(variable_voltage) <= max_abs_voltage + Util.kEpsilon) {
                    double accel = 0.0;
                    if (Double.isInfinite(curvature)) {
                        accel = (left ? -1.0 : 1.0) * (fixed_torque - variable_torque) / (angular_inertia_ *
                                wheel_radius_);
                    } else {
                        accel = (fixed_torque + variable_torque) / (linear_inertia_ * wheel_radius_);
                    }
                    result.min = Math.min(result.min, accel);
                    result.max = Math.max(result.max, accel);
                }
            }
        }
        return result;
    }

    // Can refer to velocity or acceleration depending on context.
    public static class ChassisState {
        public double linear;
        public double angular;

        public ChassisState(double linear, double angular) {
            this.linear = linear;
            this.angular = angular;
        }

        public ChassisState() {
        }

        @Override
        public String toString() {
            DecimalFormat fmt = new DecimalFormat("#0.000");
            return fmt.format(linear) + ", " + fmt.format(angular);
        }
    }

    // Can refer to velocity, acceleration, torque, voltage, etc., depending on context.
    public static class WheelState {
        public double left;
        public double right;

        public WheelState(double left, double right) {
            this.left = left;
            this.right = right;
        }

        public WheelState() {
        }

        public double get(boolean get_left) {
            return get_left ? left : right;
        }

        public void set(boolean set_left, double val) {
            if (set_left) {
                left = val;
            } else {
                right = val;
            }
        }

        @Override
        public String toString() {
            DecimalFormat fmt = new DecimalFormat("#0.000");
            return fmt.format(left) + ", " + fmt.format(right);
        }
    }

    // Full state dynamics of the drivetrain.
    // TODO maybe make these all optional fields and have a single solveDynamics() method that fills in the blanks?
    public static class DriveDynamics implements CSVWritable {
        public ChassisState chassis_velocity = new ChassisState();  // m/s
        public ChassisState chassis_acceleration = new ChassisState();  // m/s^2
        public WheelState wheel_velocity = new WheelState();  // rad/s
        public WheelState wheel_acceleration = new WheelState();  // rad/s^2
        public WheelState voltage = new WheelState();  // V
        public WheelState wheel_torque = new WheelState();  // N m

        @Override
        public String toCSV() {
            return chassis_velocity + ", " + chassis_acceleration + ", " + wheel_velocity + ", " + wheel_acceleration
                    + ", " + voltage + ", " + wheel_torque;
        }
    }
}
