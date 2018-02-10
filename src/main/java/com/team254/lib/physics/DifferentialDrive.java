package com.team254.lib.physics;

public class DifferentialDrive {
    // All units must be SI!
    protected final double linear_load_;  // kg, mass as seen by the wheels (includes both chassis mass and inertia in drivetrain)
    protected final double angular_load_;  // kg, equivalent mass as seen by the wheels (includes chassis rotational inertia and inertia in drivetrain)
    protected final double wheel_radius_;  // m
    protected final double effective_wheelbase_radius_;  // m
    protected final DCMotorTransmission left_transmission_;
    protected final DCMotorTransmission right_transmission_;
    public DifferentialDrive(final double linear_load,
                             final double angular_load,
                             final double wheel_radius,
                             final double effective_wheelbase_radius,
                             final DCMotorTransmission left_transmission,
                             final DCMotorTransmission right_transmission) {
        linear_load_ = linear_load;
        angular_load_ = angular_load;
        wheel_radius_ = wheel_radius;
        effective_wheelbase_radius_ = effective_wheelbase_radius;
        left_transmission_ = left_transmission;
        right_transmission_ = right_transmission;
    }

    // Input/output could be either velocity or acceleration...the math is the same.
    public ChassisState solveForwardKinematics(final WheelState wheel_motion) {
        ChassisState chassis_motion = new ChassisState();
        chassis_motion.linear = wheel_radius_ * (wheel_motion.right + wheel_motion.left) / 2.0;
        chassis_motion.angular = wheel_radius_ * (wheel_motion.right - wheel_motion.left) / effective_wheelbase_radius_;
        return chassis_motion;
    }

    // Input/output could be either velocity or acceleration...the math is the same.
    public WheelState solveInverseKinematics(final ChassisState chassis_motion) {
        WheelState wheel_motion = new WheelState();
        wheel_motion.left = (2.0 * chassis_motion.linear - effective_wheelbase_radius_ * chassis_motion.angular) / (2.0 * wheel_radius_);
        wheel_motion.right = (2.0 * chassis_motion.linear + effective_wheelbase_radius_ * chassis_motion.angular) / (2.0 * wheel_radius_);
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

    public void solveForwardDynamics(DriveDynamics dynamics) {
        // TODO deal with the case that one side is moving but the other side is stationary and has not broken static friction (so the whole robot pivots about the stationary side).
        dynamics.wheel_torque.left = left_transmission_.getTorqueForVoltage(dynamics.wheel_velocity.left, dynamics.voltage.left);
        dynamics.wheel_torque.right = right_transmission_.getTorqueForVoltage(dynamics.wheel_velocity.right, dynamics.voltage.right);

        dynamics.chassis_acceleration.linear = (dynamics.wheel_torque.right + dynamics.wheel_torque.left) / (wheel_radius_ * linear_load_);
        dynamics.chassis_acceleration.angular = (dynamics.wheel_torque.right - dynamics.wheel_torque.left) / (wheel_radius_ * angular_load_ * effective_wheelbase_radius_);

        dynamics.wheel_acceleration.left = dynamics.chassis_acceleration.linear - dynamics.chassis_acceleration.angular * effective_wheelbase_radius_;
        dynamics.wheel_acceleration.right = dynamics.chassis_acceleration.linear + dynamics.chassis_acceleration.angular * effective_wheelbase_radius_;
    }

    // Solve for torques and voltages.
    public DriveDynamics solveInverseDynamics(final ChassisState chassis_velocity, final ChassisState chassis_acceleration) {
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

    public void solveInverseDynamics(DriveDynamics dynamics) {
        dynamics.wheel_torque.left = wheel_radius_ / 2.0 * (dynamics.chassis_acceleration.linear * linear_load_ - dynamics.chassis_acceleration.angular * angular_load_ * effective_wheelbase_radius_);
        dynamics.wheel_torque.right = wheel_radius_ / 2.0 * (dynamics.chassis_acceleration.linear * linear_load_ + dynamics.chassis_acceleration.angular * angular_load_ * effective_wheelbase_radius_);

        dynamics.voltage.left = left_transmission_.getVoltageForTorque(dynamics.wheel_velocity.left, dynamics.wheel_torque.left);
        dynamics.voltage.right = right_transmission_.getVoltageForTorque(dynamics.wheel_velocity.right, dynamics.wheel_torque.right);
    }

    public double getMaxAbsVelocity(double curvature, double max_abs_voltage) {
        // TODO
        return 0.0;
    }

    public double getMinMaxAcceleration(double curvature, double com_velocity, double max_abs_voltage) {
        // TODO
        return 0.0;
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
    }

    // Full state dynamics of the drivetrain.
    // TODO maybe make these all optional fields and have a single solveDynamics() method that fills in the blanks?
    public static class DriveDynamics {
        public ChassisState chassis_velocity = new ChassisState();  // m/s
        public ChassisState chassis_acceleration = new ChassisState();  // m/s^2
        public WheelState wheel_velocity = new WheelState();  // rad/s
        public WheelState wheel_acceleration = new WheelState();  // rad/s^2
        public WheelState voltage = new WheelState();  // V
        public WheelState wheel_torque = new WheelState();  // N m
    }
}
