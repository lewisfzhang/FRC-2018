package com.team254.lib.trajectory.timing;

import com.team254.lib.geometry.ICurvature;
import com.team254.lib.geometry.IPose2d;
import com.team254.lib.physics.DifferentialDrive;

public class DifferentialDriveDynamicsConstraint<S extends IPose2d<S> & ICurvature<S>> implements TimingConstraint<S> {

    protected final DifferentialDrive drive_;
    protected final double abs_voltage_limit_;

    public DifferentialDriveDynamicsConstraint(final DifferentialDrive drive, double abs_voltage_limit) {
        drive_ = drive;
        abs_voltage_limit_ = abs_voltage_limit;
    }

    @Override
    public double getMaxVelocity(S state) {
        return drive_.getMaxAbsVelocity(state.getCurvature(), abs_voltage_limit_);
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(S state,
                                                    double velocity) {
        DifferentialDrive.MinMax min_max = drive_.getMinMaxAcceleration(new DifferentialDrive.ChassisState(velocity,
                state.getCurvature() * velocity), state.getCurvature(), abs_voltage_limit_);
        return new MinMaxAcceleration(min_max.min, min_max.max);
    }
}
