package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveTrain {
    protected double leftFrontPower = 0.0;
    protected double rightFrontPower = 0.0;
    protected double leftRearPower = 0.0;
    protected double rightRearPower = 0.0;

    protected static final boolean DRIVE_FORWARD = true;
    protected static final boolean DRIVE_REVERSE = false;
    protected static final boolean STRAFE_LEFT = true;
    protected static final boolean STRAFE_RIGHT = false;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;

    private static final double COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: ANDYMARK Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    BNO055IMU imu;
    private double angleOffset = 0;

    public void init (HardwareMap hardwareMap) {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive  = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        if (fieldCentric) {

            double currentAngle = Math.toRadians(getHeading());
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

    public void encoderDrive(LinearOpMode linearOpMode,
                             ElapsedTime runtime,
                             double speed,
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
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderStafe(LinearOpMode linearOpMode,
                             ElapsedTime runtime,
                             double speed,
                             double inches,
                             boolean direction,
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

    public void encoderStafe2(LinearOpMode linearOpMode,
                             ElapsedTime runtime,
                             double speed,
                             double inches,
                             boolean direction,
                             double timeoutS) {
        int newTargetPosition;
        int scale;

        if (direction == STRAFE_LEFT)
            scale = -1;
        else
            scale = 1;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {
            // Determine new target position
            newTargetPosition = leftFrontDrive.getCurrentPosition() + (int) (scale * inches * COUNTS_PER_INCH);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();

            leftFrontDrive.setPower(scale * speed);
            rightFrontDrive.setPower(scale * (-speed));
            leftRearDrive.setPower(scale * (-speed));
            rightRearDrive.setPower(scale * speed);

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {

                if (direction == STRAFE_RIGHT) {
                    if ((leftFrontDrive.getCurrentPosition()) > newTargetPosition) {
                        break;
                    }
                } else {
                    if ((leftFrontDrive.getCurrentPosition()) < newTargetPosition) {
                        break;
                    }
                }
            }

            // Stop all motion;
            stop();
        }
    }

    public void gyroStrafe(LinearOpMode linearOpMode,
                           ElapsedTime runtime,
                           DistanceSensor distanceSensor,
                           double speed,
                           double inches,
                           boolean direction,
                           double angle,
                           double timeoutS) {

        int scale;
        double currentAngle;
        double correction = 0.0;

        if (direction == STRAFE_LEFT)
            scale = -1;
        else
            scale = 1;

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // reset the timeout time and start motion.
        runtime.reset();

        while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (distanceSensor.getDistance(DistanceUnit.INCH) > inches)) {

/*            currentAngle = getHeading();
            correction = 0.0;

            if (currentAngle > angle){
                correction -= 0.2;
            } else {
                correction += 0.2;
            }
*/
            leftFrontDrive.setPower(scale*speed + correction);
            rightFrontDrive.setPower(scale*(-speed) + correction);
            leftRearDrive.setPower(scale*(-speed) + correction);
            rightRearDrive.setPower(scale*speed + correction);
        }

        // Stop all motion;
        stop();
    }

    public void gyroDrive(LinearOpMode linearOpMode,
                          ElapsedTime runtime,
                          double speed,
                          double inches,
                          double angle,
                          double timeoutS) {

        int newTargetPosition;
        int distanceRemaining;
        int stopDistance = (int)(1 * COUNTS_PER_INCH);
        boolean direction;
        double newSpeed;

        if (inches > 0) {
            direction = DRIVE_FORWARD;
            newSpeed = speed;
        } else {
            direction = DRIVE_REVERSE;
            newSpeed = -Math.abs(speed);  // Set the speed negative if driving in reverse
        }

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTargetPosition = leftFrontDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {

                distanceRemaining = Math.abs(newTargetPosition - leftFrontDrive.getCurrentPosition());
                if (distanceRemaining < stopDistance) {
                    break;
                }
                if (distanceRemaining < (6 * COUNTS_PER_INCH)) {
                    if (direction == DRIVE_FORWARD) {
                        newSpeed = 0.2;
                    } else {
                        newSpeed = -0.2;
                    }
                }

                double P_DRIVE_COEFF = 0.06;     // Larger is more responsive, but also less stable
                double error = getError(angle);
                double steer = getSteer(error, P_DRIVE_COEFF);

                double leftSpeed = newSpeed - steer;
                double rightSpeed = newSpeed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftFrontDrive.setPower(leftSpeed);
                rightFrontDrive.setPower(rightSpeed);
                leftRearDrive.setPower(leftSpeed);
                rightRearDrive.setPower(rightSpeed);
            }

            stop();
        }
    }

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
}
