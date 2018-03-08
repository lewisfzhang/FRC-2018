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

    enum DesiredMode {
        DO_NOTHING,
        CROSS_AUTO_LINE,
        SIMPLE_SWITCH,
        SCALE_AND_SWITCH,
        ONLY_SCALE,
        ADVANCED, // This uses 4 additional sendable choosers to pick one for each field state combo
    };
    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedPosition = null;

    private Optional<AutoModeCreator> mCreator = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mPositionChooser;

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.addDefault("Cross Auto Line", DesiredMode.CROSS_AUTO_LINE);
        mModeChooser.addObject("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addObject("Simple switch", DesiredMode.SIMPLE_SWITCH);
        mModeChooser.addObject("Scale AND Switch", DesiredMode.SCALE_AND_SWITCH);
        mModeChooser.addObject("Only Scale", DesiredMode.ONLY_SCALE);
        SmartDashboard.putData("Auto mode", mModeChooser);

        mPositionChooser = new SendableChooser<>();
        mPositionChooser.addDefault("Right", StartingPosition.RIGHT);
        mPositionChooser.addObject("Center", StartingPosition.CENTER);
        mPositionChooser.addObject("Left", StartingPosition.LEFT);
        SmartDashboard.putData("Starting Position", mPositionChooser);

    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition position = mPositionChooser.getSelected();
        if(mCachedDesiredMode != desiredMode || position != mCachedPosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name() + ", position->" + position.name());
            mCreator = getCreatorForParams(desiredMode, position);
        }
        mCachedDesiredMode = desiredMode;
        mCachedPosition = position;
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

    public void reset() {
        mCreator = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mModeChooser.getSelected().name());
        SmartDashboard.putString("StartingPositionSelected", mPositionChooser.getSelected().name());
    }

    public Optional<AutoModeBase> getAutoMode(AutoFieldState fieldState) {
        if (!mCreator.isPresent()) {
            return Optional.empty();
        }
        return Optional.of(mCreator.get().getStateDependentAutoMode(fieldState));
    }


}
