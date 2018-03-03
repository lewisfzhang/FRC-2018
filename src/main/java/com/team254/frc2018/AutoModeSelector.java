package com.team254.frc2018;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.creators.AutoModeCreator;
import com.team254.frc2018.auto.creators.SimpleSwitchModeCreator;
import com.team254.frc2018.auto.creators.SwitchAndScaleAutoModeCreator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {

    enum DesiredMode {
        DO_NOTHING,
        SIMPLE_SWITCH,
        SCALE_AND_SWITCH,
        ONLY_SCALE,
        ADVANCED, // This uses 4 additional sendable choosers to pick one for each field state combo
    };
    private DesiredMode mCachedDesiredMode = null;

    private Optional<AutoModeCreator> mCreator = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;

    public AutoModeSelector() {
        mModeChooser.addDefault("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addObject("Simple switch", DesiredMode.SIMPLE_SWITCH);
        mModeChooser.addObject("Scale AND Switch", DesiredMode.SCALE_AND_SWITCH);
        mModeChooser.addObject("Only Scale", DesiredMode.ONLY_SCALE);
        SmartDashboard.putData("Auto mode", mModeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        if(mCachedDesiredMode != desiredMode) {
            System.out.println("Auto selection changed, updating creator" + desiredMode);
            mCreator = getCreatorForParams(desiredMode);
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeCreator> getCreatorForParams(DesiredMode mode) {
        switch (mode) {
            case SIMPLE_SWITCH:
                return Optional.of(new SimpleSwitchModeCreator());
            case SCALE_AND_SWITCH:
                return Optional.of(new SwitchAndScaleAutoModeCreator(false));
                // TODO: Make this work for th
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
        SmartDashboard.putString("AutoModeSelected", mModeChooser.getSelected().toString());
    }

    public Optional<AutoModeBase> getAutoMode(AutoFieldState fieldState) {
        if (!mCreator.isPresent()) {
            return Optional.empty();
        }
        return Optional.of(mCreator.get().getStateDependentAutoMode(fieldState));
    }


}
