package com.team254.frc2018.paths;

import com.team254.frc2018.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 120.0;
    private static final double kMaxAccel = 100.0;
    private static final double kMaxCentripetalAccelElevatorDown = 100.0;
    private static final double kMaxCentripetalAccel = 80.0;
    private static final double kMaxVoltage = 9.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if(mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)
    public static final Pose2d kSideStartPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kNearScaleEmptyPose = new Pose2d(new Translation2d(250.0, 28.0), Rotation2d
            .fromDegrees(10.0+180.0));
    public static final Pose2d kNearScaleFullPose = new Pose2d(new Translation2d(250.0, 28.0), Rotation2d.fromDegrees
            (15.0+180.0));

    public static final Pose2d kNearScaleFullPose1 = new Pose2d(new Translation2d(250.0, 28.0 + 2.0), Rotation2d.fromDegrees
            (15.0+180.0));

    public static final Pose2d kNearScaleFullPose2 = new Pose2d(new Translation2d(250.0, 28.0 + 4.0), Rotation2d.fromDegrees
            (15.0+180.0));

    public static final Pose2d kNearScaleEndPose = new Pose2d(new Translation2d(220.0, 0.0), Rotation2d.fromDegrees
            (45.0));

    public static final Pose2d kFarScaleEmptyPose = new Pose2d(new Translation2d(250.0, 201.0), Rotation2d
            .fromDegrees(-10.0+180.0));
    public static final Pose2d kFarScaleFullPose = new Pose2d(new Translation2d(250.0, 202.0), Rotation2d.fromDegrees
            (-15.0+180.0));
    public static final Pose2d kFarScaleFullPose1 = new Pose2d(new Translation2d(250.0, 202.0 - 2.0), Rotation2d.fromDegrees
            (-15.0+180.0));
    public static final Pose2d kFarScaleFullPose2 = new Pose2d(new Translation2d(250.0, 202.0 - 4.0), Rotation2d.fromDegrees
            (-15.0+180.0));

    public static final Pose2d kCenterToIntake = new Pose2d(new Translation2d(-24.0, 0.0), Rotation2d.identity());

    public static final Pose2d kNearCube1Pose = new Pose2d(new Translation2d(182.0, 46.0), Rotation2d.fromDegrees(180.0 - 25.0));
    public static final Pose2d kNearCube2Pose = new Pose2d(new Translation2d(182.0, 46.0 + 30.0 + 12.0), Rotation2d.fromDegrees(180.0 - 65.0));
    public static final Pose2d kNearCube3Pose = new Pose2d(new Translation2d(179.0, 46.0 + 30.0 * 2 + 12.0), Rotation2d.fromDegrees(180.0 - 65.0));

    public static final Pose2d kNearFence1Pose = kNearCube1Pose.transformBy(kCenterToIntake);
    public static final Pose2d kNearFence2Pose = kNearCube2Pose.transformBy(kCenterToIntake);
    public static final Pose2d kNearFence3Pose = kNearCube3Pose.transformBy(kCenterToIntake);

    public static final Pose2d kFarCube1Pose = new Pose2d(new Translation2d(183.0, 187.0), Rotation2d.fromDegrees(180.0 + 25.0));
    public static final Pose2d kFarCube2Pose = new Pose2d(new Translation2d(184.5, 190.0 - 30.0 - 12.0), Rotation2d.fromDegrees(180.0 + 65.0));
    public static final Pose2d kFarCube3Pose = new Pose2d(new Translation2d(179.0, 204.0 - 30.0 * 2 - 12.0), Rotation2d.fromDegrees(180.0 + 65.0));

    public static final Pose2d kFarFence1Pose = kFarCube1Pose.transformBy(kCenterToIntake);
    public static final Pose2d kFarFence2Pose = kFarCube2Pose.transformBy(kCenterToIntake);
    public static final Pose2d kFarFence3Pose = kFarCube3Pose.transformBy(kCenterToIntake);

    // STARTING IN CENTER
    public static final Pose2d kCenterStartPose = new Pose2d(0.0, -2.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kLeftSwitchPose = new Pose2d(new Translation2d(100.0, 60.0), Rotation2d.fromDegrees(0.0+180.0));
    public static final Pose2d kRightSwitchPose = new Pose2d(new Translation2d(100.0, -60.0), Rotation2d.fromDegrees(0.0+180.0));

    public static final Pose2d kPyramidCubePose = new Pose2d(new Translation2d(82.0, 5.0), Rotation2d.fromDegrees(0.0 + 60.0))
            .transformBy(kCenterToIntake);
    public static final Pose2d kPyramidCube1Pose = new Pose2d(new Translation2d(82.0 + 15, 6.0), Rotation2d.fromDegrees(0.0 + 60.0))
            .transformBy(kCenterToIntake);
    public static final Pose2d kPyramidCube2Pose = new Pose2d(new Translation2d(82.0 + 14.0, 5.0 - 2.0), Rotation2d.fromDegrees(0.0 + 60.0))
            .transformBy(kCenterToIntake);

    public static final Pose2d kSimpleSwitchEndPose = new Pose2d(new Translation2d(200, -120.0), Rotation2d.fromDegrees(180.0 + 25.0))
            .transformBy(kCenterToIntake);

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        public final MirroredTrajectory sideStartToNearScale;
        public final MirroredTrajectory sideStartToFarScale;
        public final MirroredTrajectory sideStartToNearSwitch;
        public final MirroredTrajectory sideStartToFarSwitch;
        public final MirroredTrajectory nearScaleToNearFence;
        public final MirroredTrajectory nearScaleToNearFence2;
        public final MirroredTrajectory nearScaleToNearFence3;
        public final MirroredTrajectory nearScaleToFarFence;
        public final MirroredTrajectory nearFenceToNearScale;
        public final MirroredTrajectory nearFence2ToNearScale;
        public final MirroredTrajectory nearFence3ToNearScale;
        public final MirroredTrajectory nearFence3ToEndPose;
        public final MirroredTrajectory farScaleToFarFence;
        public final MirroredTrajectory farScaleToFarFence2;
        public final MirroredTrajectory farScaleToFarFence3;
        public final MirroredTrajectory farFenceToFarScale;
        public final MirroredTrajectory farFence2ToFarScale;
        public final MirroredTrajectory farFence3ToFarScale;
        public final MirroredTrajectory nearFenceToFarSwitch;
        public final MirroredTrajectory farFenceToNearSwitch;

        public final Trajectory<TimedState<Pose2dWithCurvature>> centerStartToLeftSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> centerStartToRightSwitch;
        public final MirroredTrajectory switchToPyramidCube;
        public final MirroredTrajectory pyramidCubeToSwitch;
        public final MirroredTrajectory switchToPyramidCube1;
        public final MirroredTrajectory pyramidCube1ToSwitch;
        public final MirroredTrajectory switchToPyramidCube2;
        public final MirroredTrajectory pyramidCube2ToCenterField;

        private TrajectorySet() {
            sideStartToNearScale = new MirroredTrajectory(getSideStartToNearScale());
            nearScaleToNearFence = new MirroredTrajectory(getNearScaleToNearFence());
            nearScaleToNearFence2 = new MirroredTrajectory(getNearScaleToNearFence2());
            nearScaleToNearFence3 = new MirroredTrajectory(getNearScaleToNearFence3());
            nearScaleToFarFence = new MirroredTrajectory(getNearScaleToFarFence());
            sideStartToFarScale = new MirroredTrajectory(getSideStartToFarScale());
            nearFenceToNearScale = new MirroredTrajectory(getNearFenceToNearScale());
            nearFence2ToNearScale = new MirroredTrajectory(getNearFence2ToNearScale());
            nearFence3ToNearScale = new MirroredTrajectory(getNearFence3ToNearScale());
            nearFence3ToEndPose = new MirroredTrajectory(getNearFence3ToEndPose());
            farScaleToFarFence = new MirroredTrajectory(getFarScaleToFarFence());
            farScaleToFarFence2 = new MirroredTrajectory(getFarScaleToFarFence2());
            farScaleToFarFence3 = new MirroredTrajectory(getFarScaleToFarFence3());
            farFenceToFarScale = new MirroredTrajectory(getFarFenceToFarScale());
            farFence2ToFarScale = new MirroredTrajectory(getFarFence2ToFarScale());
            farFence3ToFarScale = new MirroredTrajectory(getFarFence3ToFarScale());
            sideStartToNearSwitch = new MirroredTrajectory(getSideStartToNearSwitch());
            sideStartToFarSwitch = new MirroredTrajectory(getSideStartToFarSwitch());
            nearFenceToFarSwitch = new MirroredTrajectory(getNearFenceToFarSwitch());
            farFenceToNearSwitch = new MirroredTrajectory(getFarFenceToNearSwitch());

            centerStartToLeftSwitch = getCenterStartToLeftSwitch();
            centerStartToRightSwitch = getCenterStartToRightSwitch();
            switchToPyramidCube = new MirroredTrajectory(getSwitchToPyramidCube());
            pyramidCubeToSwitch = new MirroredTrajectory(getPyramidCubeToSwitch());
            switchToPyramidCube1 = new MirroredTrajectory(getSwitchToPyramidCube1());
            pyramidCube1ToSwitch = new MirroredTrajectory(getPyramidCube1ToSwitch());
            switchToPyramidCube2 = new MirroredTrajectory(getSwitchToPyramidCube2());
            pyramidCube2ToCenterField = new MirroredTrajectory(getPyramidCube2ToCenterField());
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToNearScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSideStartPose);
            waypoints.add(kSideStartPose.transformBy(Pose2d.fromTranslation(new Translation2d(-120.0, 0.0))));
            waypoints.add(kNearScaleEmptyPose);
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccelElevatorDown)),
                    120.0, 140.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearScaleToNearFence() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kNearScaleEmptyPose);
            waypoints.add(kNearFence1Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearScaleToNearFence2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kNearScaleFullPose);
            waypoints.add(kNearFence2Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearScaleToNearFence3() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kNearScaleFullPose1);
            waypoints.add(kNearFence3Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToNearSwitch() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSideStartPose);
            waypoints.add(new Pose2d(new Translation2d(135.0, 36.0), Rotation2d.fromDegrees(270.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        // TODO tune
        private Trajectory<TimedState<Pose2dWithCurvature>> getNearScaleToFarFence() {
            List<Pose2d> waypoints = new ArrayList<>();
            // TODO these are wrong!
            waypoints.add(new Pose2d(new Translation2d(-250.0, -200.0), Rotation2d.fromDegrees(170.0)));
            waypoints.add(new Pose2d(new Translation2d(-208.0, -190.0), Rotation2d.fromDegrees(225.0)).transformBy
                    (Pose2d.fromTranslation(new Translation2d(-12.0, 0.0))));
            waypoints.add(new Pose2d(new Translation2d(-208.0, -190.0), Rotation2d.fromDegrees(225.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), 45,
                    45, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToFarScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSideStartPose);
            waypoints.add(kSideStartPose.transformBy(new Pose2d(new Translation2d(-160.0, 0.0), Rotation2d
                    .fromDegrees(0.0))));
            waypoints.add(kSideStartPose.transformBy(new Pose2d(new Translation2d(-220.0, -60.0), Rotation2d
                    .fromDegrees(90.0))));
            waypoints.add(kSideStartPose.transformBy(new Pose2d(new Translation2d(-220.0, -180.0), Rotation2d
                    .fromDegrees(90.0))));
            // waypoints.add(kSideStartPose.transformBy(new Pose2d(new Translation2d(250.0, 205.0), Rotation2d
            // .fromDegrees(-10.0))));
            waypoints.add(kFarScaleEmptyPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccelElevatorDown)),
                    120.0, 140.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToFarSwitch() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(105.0, 180.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarScaleToFarFence() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kFarScaleEmptyPose);
            waypoints.add(kFarFence1Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxAccel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarScaleToFarFence2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kFarScaleFullPose);
            waypoints.add(kFarFence2Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarScaleToFarFence3() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kFarScaleFullPose1);
            waypoints.add(kFarFence3Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarFenceToFarScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kFarFence1Pose);
            waypoints.add(kFarScaleFullPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxAccel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarFence2ToFarScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kFarFence2Pose);
            waypoints.add(kFarScaleFullPose1);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarFence3ToFarScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kFarFence3Pose);
            waypoints.add(kFarScaleFullPose2);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearFenceToNearScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kNearFence1Pose);
            waypoints.add(kNearScaleFullPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearFence2ToNearScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kNearFence2Pose);
            waypoints.add(kNearScaleFullPose1);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearFence3ToNearScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kNearFence3Pose);
            waypoints.add(kNearScaleFullPose2);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearFence3ToEndPose() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kNearFence3Pose);
            waypoints.add(kNearScaleEndPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarFenceToNearSwitch() {
            List<Pose2d> waypoints = new ArrayList<>();

            waypoints.add(kFarFence1Pose);
            waypoints.add(new Pose2d(new Translation2d(230, 127.0), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(230, 67.0), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(205, 47.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearFenceToFarSwitch() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kNearFence1Pose);
            waypoints.add(new Pose2d(new Translation2d(230, 100), Rotation2d.fromDegrees(-90)));
            waypoints.add(new Pose2d(new Translation2d(230, 130), Rotation2d.fromDegrees(-90)));
            waypoints.add(new Pose2d(new Translation2d(205, 170), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }



        private Trajectory<TimedState<Pose2dWithCurvature>> getCenterStartToLeftSwitch() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCenterStartPose);
            waypoints.add(kLeftSwitchPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCenterStartToRightSwitch() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCenterStartPose);
            waypoints.add(kRightSwitchPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSwitchToPyramidCube() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightSwitchPose);
            waypoints.add(kPyramidCubePose.transformBy(Pose2d.fromTranslation(new Translation2d(-12.0, 0.0))));
            waypoints.add(kPyramidCubePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getPyramidCubeToSwitch() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kPyramidCubePose);
            waypoints.add(kRightSwitchPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSwitchToPyramidCube1() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightSwitchPose);
            waypoints.add(kPyramidCube1Pose.transformBy(Pose2d.fromTranslation(new Translation2d(-12.0, 0.0))));
            waypoints.add(kPyramidCube1Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getPyramidCube1ToSwitch() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kPyramidCube1Pose);
            waypoints.add(kRightSwitchPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSwitchToPyramidCube2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightSwitchPose);
            waypoints.add(kPyramidCube2Pose.transformBy(Pose2d.fromTranslation(new Translation2d(-12.0, 0.0))));
            waypoints.add(kPyramidCube2Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getPyramidCube2ToCenterField() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kPyramidCube1Pose);
            waypoints.add(kSimpleSwitchEndPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

    }
}
