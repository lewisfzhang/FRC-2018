package com.team254.frc2018.subsystems;

import com.ctre.phoenix.CANifier;
import com.team254.frc2018.Constants;

public class CarriageCanifier {
    private static CarriageCanifier mInstance;

    private CANifier mCanifier;

    private CarriageCanifier() {
        mCanifier = new CANifier(Constants.kCanifierId);
    }

    public synchronized static CarriageCanifier getInstance() {
        if (mInstance == null) {
            mInstance = new CarriageCanifier();
        }
        return mInstance;
    }

    public boolean getLeftBannerSensor() {
        return !mCanifier.getGeneralInput(CANifier.GeneralPin.LIMF);
    }

    public boolean getRightBannerSensor() {
        return !mCanifier.getGeneralInput(CANifier.GeneralPin.LIMR);
    }

    public synchronized void setLEDColor(double red, double green, double blue) {
        // A: Blue
        // B: Green
        // C: Red
        mCanifier.setLEDOutput(blue, CANifier.LEDChannel.LEDChannelA);
        mCanifier.setLEDOutput(green, CANifier.LEDChannel.LEDChannelB);
        mCanifier.setLEDOutput(red, CANifier.LEDChannel.LEDChannelC);
    }


}
