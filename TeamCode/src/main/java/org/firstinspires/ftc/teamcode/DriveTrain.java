package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveTrain {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;

    public double leftFrontPower = 0.0;
    public double rightFrontPower = 0.0;
    public double leftRearPower = 0.0;
    public double rightRearPower = 0.0;

    protected BNO055IMU imu;
    private double angleOffset = 0;


    private static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: ANDYMARK Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public void init (HardwareMap hardwareMap) {

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive  = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void drive(double left_x, double left_y, double right_x, boolean fieldCentric) {
        double currentAngle = Math.toRadians(getHeading());
        if (fieldCentric) {

            double new_x = left_x * Math.cos(currentAngle) - left_y * Math.sin(currentAngle);
            double new_y = left_x * Math.sin(currentAngle) + left_y * Math.cos(currentAngle);

            left_x = new_x;
            left_y = new_y;
        }

        leftFrontPower   = Range.clip(left_y + right_x + left_x, -1.0, 1.0) ;
        rightFrontPower  = Range.clip(left_y - right_x - left_x, -1.0, 1.0) ;
        leftRearPower    = Range.clip(left_y + right_x - left_x, -1.0, 1.0) ;
        rightRearPower   = Range.clip(left_y - right_x + left_x, -1.0, 1.0) ;

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);
    }

    public void stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
    }

    public void resetAngle() {
        Orientation currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleOffset = -((AngleUnit.DEGREES.normalize(currentAngles.firstAngle))+ 360) %360;
    }

    public double getHeading() {
        Orientation currentAngles;
        double heading;

        currentAngles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = -((AngleUnit.DEGREES.normalize(currentAngles.firstAngle))+ angleOffset + 360) %360;
        return heading;
    }
    public double getHeading2() {
        Orientation currentAngles;
        double heading;

        currentAngles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = ((AngleUnit.DEGREES.normalize(currentAngles.firstAngle)));
        return heading;
    }



    public void rotate(LinearOpMode linearOpMode, double desiredAngle, double speed) {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //slowed down speed to be more accurate in angle turns
        leftFrontDrive.setPower(-speed);
        rightFrontDrive.setPower(speed);
        leftRearDrive.setPower(-speed);
        rightRearDrive.setPower(speed);

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if(speed > 0) {
            while(!linearOpMode.isStopRequested() && desiredAngle >= angles.firstAngle) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                linearOpMode.telemetry.addData("Gyro","DesiredAngle: %.1f, Current Angle: %.1f", desiredAngle, AngleUnit.DEGREES.normalize(angles.firstAngle));
                linearOpMode.telemetry.update();
            }
        } else {
            while (!linearOpMode.isStopRequested() && desiredAngle <= angles.firstAngle) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                linearOpMode.telemetry.addData("Gyro", "DesiredAngle: %.1f, Current Angle: %.1f", desiredAngle, AngleUnit.DEGREES.normalize(angles.firstAngle));
                linearOpMode.telemetry.update();
            }
        }
        stop();
    }

    public void encoderDrive(LinearOpMode linearOpMode, ElapsedTime runtime, double speed,
                             double inches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftRearTarget = leftRearDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightRearTarget = rightRearDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            leftRearDrive.setTargetPosition(newLeftRearTarget);
            rightRearDrive.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            leftRearDrive.setPower(Math.abs(speed));
            rightRearDrive.setPower(Math.abs(speed));

            while (linearOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftRearDrive.isBusy() && rightRearDrive.isBusy())) {
            }

            // Stop all motion;
            stop();

            // Turn off RUN_TO_POSITION
            // leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderStafe(LinearOpMode linearOpMode, ElapsedTime runtime, double speed,
                             double inches, boolean direction,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {
            if(direction == false) {
                // Determine new target position, and pass to motor controller
                newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                newRightFrontTarget = rightFrontDrive.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                newLeftRearTarget = leftRearDrive.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                newRightRearTarget = rightRearDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

                leftFrontDrive.setTargetPosition(newLeftFrontTarget);
                rightFrontDrive.setTargetPosition(newRightFrontTarget);
                leftRearDrive.setTargetPosition(newLeftRearTarget);
                rightRearDrive.setTargetPosition(newRightRearTarget);

                // Turn On RUN_TO_POSITION
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                leftFrontDrive.setPower(Math.abs(speed));
                rightFrontDrive.setPower(-Math.abs(speed));
                leftRearDrive.setPower(-Math.abs(speed));
                rightRearDrive.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (linearOpMode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftRearDrive.isBusy() && rightRearDrive.isBusy())) {
                }

                // Stop all motion;
                stop();

                // Turn off RUN_TO_POSITION
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            } else {
                // Determine new target position, and pass to motor controller
                newLeftFrontTarget = leftFrontDrive.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                newLeftRearTarget = leftRearDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                newRightRearTarget = rightRearDrive.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);

                leftFrontDrive.setTargetPosition(newLeftFrontTarget);
                rightFrontDrive.setTargetPosition(newRightFrontTarget);
                leftRearDrive.setTargetPosition(newLeftRearTarget);
                rightRearDrive.setTargetPosition(newRightRearTarget);

                // Turn On RUN_TO_POSITION
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                leftFrontDrive.setPower(-Math.abs(speed));
                rightFrontDrive.setPower(Math.abs(speed));
                leftRearDrive.setPower(Math.abs(speed));
                rightRearDrive.setPower(-Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (linearOpMode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftRearDrive.isBusy() && rightRearDrive.isBusy())) {
                }

                // Stop all motion;
                stop();

                // Turn off RUN_TO_POSITION
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            //  sleep(250);   // optional pause after each move
        }
    }
}
