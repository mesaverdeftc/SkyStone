package org.firstinspires.ftc.teamcode;

final class MotorSpeed {
    private final double leftSpeed;
    private final double rightSpeed;

    public MotorSpeed(double leftSpeed, double rightSpeed) {
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
    }

    public double getLeftSpeed() {return leftSpeed;}
    public double getRightSpeed() {return rightSpeed;}
}
