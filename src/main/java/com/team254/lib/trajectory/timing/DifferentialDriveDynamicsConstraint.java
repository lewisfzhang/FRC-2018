package com.team254.lib.trajectory.timing;

import com.team254.lib.geometry.ICurvature;
import com.team254.lib.geometry.IPose2d;

public class DifferentialDriveDynamicsConstraint<S extends IPose2d<S> & ICurvature<S>> implements TimingConstraint<S> {

    // TODO: Implement constraints based on a differential drive (ensuring that we don't exceed voltage/current
    // limits when following a trajectory).

    public DifferentialDriveDynamicsConstraint() {

    }

    @Override
    public double getMaxVelocity(S state) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public com.team254.lib.trajectory.timing.TimingConstraint.MinMaxAcceleration getMinMaxAcceleration(S state,
                                                                                                       double velocity) {
        // TODO Auto-generated method stub
        return null;
    }

}
