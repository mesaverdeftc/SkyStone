package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorDistance {

    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;

    public void init(HardwareMap hardwareMap, String deviceName) {
        // Get a reference to our sensor object.
        sensorColor = hardwareMap.get(ColorSensor.class, deviceName);
        sensorDistance = hardwareMap.get(DistanceSensor.class, deviceName);

    }

    int red() {
        return sensorColor.red();
    }

    int blue() {
        return sensorColor.blue();
    }

    int green() {
        return sensorColor.green();
    }

    int alpha() {
        return sensorColor.alpha();
    }


    double getDistance(DistanceUnit distanceUnit) {
        return sensorDistance.getDistance(distanceUnit);
    }

    // How to detect a skystone with the color sensor taken from https://www.youtube.com/watch?v=i0AskHFkZ94
    boolean isStone() {
        double r, g, b;

        r = sensorColor.red();
        g = sensorColor.green();
        b = sensorColor.blue();

        double calculation = (r * g)/ (b *b);

        if(calculation > 3) {
            return true;
        }
        else {
            return false;
        }
    }
}
