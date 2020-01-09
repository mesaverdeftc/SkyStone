package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Foundation extends Attachment{
    Servo servo2 = null;

    public void init(HardwareMap hardwareMap, String servoName, String servoName2, double upValue, double downValue) {
        servo = hardwareMap.get(Servo.class, servoName);
        servo2 = hardwareMap.get(Servo.class, servoName2);

        upPosition = upValue;
        downPosition = downValue;
    }

    @Override
    public void up() {
        servo.setPosition(upPosition);
        servo2.setPosition(-upPosition);
    }

    @Override
    public void down() {
        servo.setPosition(downPosition);
        servo2.setPosition(-downPosition);
    }
}
