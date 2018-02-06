package com.team254.lib.geometry;

import com.team254.lib.util.Util;

import java.text.DecimalFormat;

public class Pose2dWithCurvature implements IPose2d<Pose2dWithCurvature>, ICurvature<Pose2dWithCurvature> {
    protected static final Pose2dWithCurvature kIdentity = new Pose2dWithCurvature();

    public static final Pose2dWithCurvature identity() {
        return kIdentity;
    }

    protected Pose2d pose_;
    protected double curvature_;

    public Pose2dWithCurvature() {
        pose_ = new Pose2d();
        curvature_ = 0.0;
    }

    public Pose2dWithCurvature(final Pose2d pose, double curvature) {
        pose_ = pose;
        curvature_ = curvature;
    }

    @Override
    public final Pose2d getPose() {
        return pose_;
    }

    @Override
    public double getCurvature() {
        return curvature_;
    }

    @Override
    public final Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    @Override
    public final Rotation2d getRotation() {
        return getPose().getRotation();
    }

    @Override
    public Pose2dWithCurvature interpolate(final Pose2dWithCurvature other, double x) {
        return new Pose2dWithCurvature(getPose().interpolate(other.getPose(), x),
                Util.interpolate(getCurvature(), other.getCurvature(), x));
    }

    @Override
    public double distance(final Pose2dWithCurvature other) {
        return getPose().distance(other.getPose());
    }

    @Override
    public boolean isEqual(final Pose2dWithCurvature other) {
        return getPose().isEqual(other.getPose()) && Util.epsilonEquals(getCurvature(), other.getCurvature());
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return getPose().toString() + ", curvature: " + fmt.format(getCurvature());
    }

    @Override
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return getPose().toCSV() + "," + fmt.format(getCurvature());
    }
}