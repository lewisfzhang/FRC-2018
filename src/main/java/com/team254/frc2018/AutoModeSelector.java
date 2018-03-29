package com.team254.frc2018;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.creators.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {

    enum StartingPosition {
        LEFT,
        CENTER,
        RIGHT,
    };

    enum SwitchScalePosition {
        USE_FMS_DATA,
        LEFT_SWITCH_LEFT_SCALE,
        LEFT_SWITCH_RIGHT_SCALE,
        RIGHT_SWITCH_LEFT_SCALE,
        RIGHT_SWITCH_RIGHT_SCALE,
    };

    enum DesiredMode {
        DO_NOTHING,
        CROSS_AUTO_LINE,
        SIMPLE_SWITCH,
        SCALE_AND_SWITCH,
        ONLY_SCALE,
        ADVANCED, // This uses 4 additional sendable choosers to pick one for each field state combo
    };

    private DesiredMode mCachedDesiredMode = null;
    private SwitchScalePosition mCachedSwitchScalePosition = null;
    private StartingPosition mCachedStartingPosition = null;

    private Optional<AutoFieldState> mOverrideFieldState = Optional.empty();
    private Optional<AutoModeCreator> mCreator = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<SwitchScalePosition> mSwitchScalePositionChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.addDefault("Cross Auto Line", DesiredMode.CROSS_AUTO_LINE);
        mModeChooser.addObject("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addObject("Simple switch", DesiredMode.SIMPLE_SWITCH);
        mModeChooser.addObject("Scale AND Switch", DesiredMode.SCALE_AND_SWITCH);
        mModeChooser.addObject("Only Scale", DesiredMode.ONLY_SCALE);
        SmartDashboard.putData("Auto mode", mModeChooser);

        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.addDefault("Right", StartingPosition.RIGHT);
        mStartPositionChooser.addObject("Center", StartingPosition.CENTER);
        mStartPositionChooser.addObject("Left", StartingPosition.LEFT);
        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mSwitchScalePositionChooser = new SendableChooser<>();
        mSwitchScalePositionChooser.addDefault("Use FMS Data", SwitchScalePosition.USE_FMS_DATA);
        mSwitchScalePositionChooser.addObject("Left Switch Left Scale", SwitchScalePosition.LEFT_SWITCH_LEFT_SCALE);
        mSwitchScalePositionChooser.addObject("Left Switch Right Scale", SwitchScalePosition.LEFT_SWITCH_RIGHT_SCALE);
        mSwitchScalePositionChooser.addObject("Right Switch Left Scale", SwitchScalePosition.RIGHT_SWITCH_LEFT_SCALE);
        mSwitchScalePositionChooser.addObject("Right Switch Right Scale", SwitchScalePosition.RIGHT_SWITCH_RIGHT_SCALE);
        SmartDashboard.putData("Switch and Scale Position", mSwitchScalePositionChooser);

    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition staringPosition = mStartPositionChooser.getSelected();
        SwitchScalePosition switchScalePosition = mSwitchScalePositionChooser.getSelected();
        if(mCachedDesiredMode != desiredMode || staringPosition != mCachedStartingPosition || switchScalePosition != mCachedSwitchScalePosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name() + ", starting position->" + staringPosition.name() + ", switch/scale position->" + switchScalePosition.name());
            mCreator = getCreatorForParams(desiredMode, staringPosition);
            mOverrideFieldState = getFieldStateForParams(switchScalePosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = staringPosition;
        mCachedSwitchScalePosition = switchScalePosition;
    }

    private Optional<AutoModeCreator> getCreatorForParams(DesiredMode mode, StartingPosition position) {
        boolean startOnLeft = StartingPosition.LEFT == position;
        switch (mode) {
            case SIMPLE_SWITCH:
                return Optional.of(new SimpleSwitchModeCreator());
            case CROSS_AUTO_LINE:
                return Optional.of(new CrossAutoLineCreator());
            case SCALE_AND_SWITCH:
                return Optional.of(new SwitchAndScaleAutoModeCreator(startOnLeft));
            case ONLY_SCALE:
                return Optional.of(new ScaleOnlyAutoModeCreator(startOnLeft));
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    private Optional<AutoFieldState> getFieldStateForParams(SwitchScalePosition switchScalePosition) {
        switch (switchScalePosition) {
            case LEFT_SWITCH_LEFT_SCALE:
                return Optional.of(new AutoFieldState("LLL"));
            case LEFT_SWITCH_RIGHT_SCALE:
                return Optional.of(new AutoFieldState("LRL"));
            case RIGHT_SWITCH_LEFT_SCALE:
                return Optional.of(new AutoFieldState("RLR"));
            case RIGHT_SWITCH_RIGHT_SCALE:
                return Optional.of(new AutoFieldState("RRR"));
            default:
                break;
        }
        return Optional.empty();
    }

    public void reset() {
        mCreator = Optional.empty();
        mOverrideFieldState = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
        SmartDashboard.putString("SwitchScalePositionSelected", mCachedSwitchScalePosition.name());
    }

    public Optional<AutoModeBase> getAutoMode(AutoFieldState fieldState) {
        if (!mCreator.isPresent()) {
            return Optional.empty();
        }
        if(mCachedSwitchScalePosition != SwitchScalePosition.USE_FMS_DATA && mOverrideFieldState.isPresent()) {
            System.out.println("Overriding FMS switch/scale positions!");
            return Optional.of(mCreator.get().getStateDependentAutoMode(mOverrideFieldState.get()));
        } else {
            return Optional.of(mCreator.get().getStateDependentAutoMode(fieldState));
        }
    }


}
