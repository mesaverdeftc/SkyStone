package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class ColorDistance {

    private ColorSensor colorSensor;

    // values is a reference to the hsvValues array.
    float hsvValues[] = {0F, 0F, 0F};
    final double SCALE_FACTOR = 255;

    public void init(HardwareMap hardwareMap, String deviceName) {
        // Get a reference to our sensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, deviceName);

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
    }

    public float [] getHSVColor() {
        // Read the sensor

        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsvValues);

        return hsvValues;
    }
}
