package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class GyroSteerCorrection {
    private static double P_DRIVE_COEFF = 0.14;
    private BNO055IMU imu;

    GyroSteerCorrection (BNO055IMU imu) {this.imu = imu;}

    public double getError(double targetAngle) {

        double robotError;

        Orientation currentAngles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - AngleUnit.DEGREES.normalize(currentAngles.firstAngle);
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;

    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public MotorSpeed correctMottorSpeed(double speed, double angle) {
        double error = getError(angle);
        double steer = getSteer(error, P_DRIVE_COEFF);

        double leftSpeed = speed - steer;
        double rightSpeed = speed + steer;

        // Normalize speeds if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }
        return new MotorSpeed(leftSpeed, rightSpeed);
    }
}
