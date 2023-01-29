package org.firstinspires.ftc.teamcode.symbiotes.vision.utility;

import androidx.annotation.NonNull;

/**
 * Enum to hold the possible signal sleeve values.
 * Ordinal of the enum correlate to the value of the signal of the field.
 * Values are the parking position of the sleeve relative to the robot facing the signal cone.
 */
public enum Signal {
    NONE,
    LEFT,
    MIDDLE,
    RIGHT;

    @NonNull
    // Get the signal value based on the position on the sleeve.
    public static Signal getSignalByOrdinal(int ordinal) {
        for(Signal s : Signal.values()) {
            if(s.ordinal() == ordinal)
                return s;
        }
        return NONE;
    }
}