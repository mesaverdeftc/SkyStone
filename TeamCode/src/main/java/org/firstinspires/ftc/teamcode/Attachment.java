package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Attachment {

    protected Servo servo = null;
    protected double upPosition = 0.0;
    protected double downPosition = 0.0;

    public void init(HardwareMap hardwareMap, String servoName, double upValue, double downValue) {
        servo = hardwareMap.get(Servo.class, servoName);

        upPosition = upValue;
        downPosition = downValue;
    }

    public void up() { servo.setPosition(upPosition); }

    public void down() { servo.setPosition(downPosition); }

    public void toggle(boolean toggleState) {
        if(toggleState) {
            up();
        } else {
            down();
        }
    }
}
