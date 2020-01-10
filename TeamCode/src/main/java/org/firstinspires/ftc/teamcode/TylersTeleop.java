/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Tyler's Teleop", group="Iterative Opmode")
@Disabled
public class TylersTeleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;
    private Servo servoBlock = null;
    private Servo servoGraber = null;
    private Servo servoFoundation1 = null;
    private Servo servoFoundation2 = null;
    private Servo servoCapstone = null;
    private double previousTime = 0.0;
    private double loopsPerSecond = 0.0;
    private double loops = 0.0;

    private int iterations0 = 1;
    private int iterations1 = 1;
    private int iterations2 = 1;
    private boolean active0 = false;
    private boolean active1 = false;
    private boolean active2 = false;

    private boolean button_rb_IsActive = false;
    private boolean slowmode = false;

    private boolean button_x_IsActive = false;
    private boolean graberClosed = false;


    // The IMU sensor object
    BNO055IMU imu;
    Orientation angles;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive  = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");

        servoBlock = hardwareMap.get(Servo.class, "block_servo0");
        servoGraber = hardwareMap.get(Servo.class, "grabber_servo4");
        servoFoundation1 = hardwareMap.get(Servo.class, "foundation_servo1");
        servoFoundation2 = hardwareMap.get(Servo.class, "foundation_servo2");
        servoCapstone = hardwareMap.get(Servo.class, "capstone_servo3");

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
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

        // Tell the driver that initialization is complete.
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double rightFrontPower;
        double leftRearPower;
        double rightRearPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.

        double left_x = gamepad1.left_stick_x;
        double left_y = -gamepad1.left_stick_y;
        double right_x = gamepad1.right_stick_x;
        boolean button_rb = gamepad1.right_bumper;

        boolean button_y = gamepad2.y;
        boolean button_x = gamepad2.x;
        boolean button_b = gamepad2.b;
        boolean button_dpad_down = gamepad2.dpad_down;

        // button Y on controller 2 controls the block grabber attachment servo
        if(button_y && !active0) {
            active0 = true;
            if (iterations0 % 2 == 0) {
                servoBlock.setPosition(-1.0);
                iterations0++;
            } else {
                servoBlock.setPosition(1.0);
                iterations0++;
            }
        }
        else if (!button_y) {
            active0 = false;
        }

        if(button_x && !button_x_IsActive) {
            button_x_IsActive = true;
            if (graberClosed) {
                servoGraber.setPosition(-1.0);
                graberClosed = false;
            } else {
                servoGraber.setPosition(1.0);
                graberClosed = true;
            }

        }
        else if (!button_x) {
            button_x_IsActive = false;
        }

        // button B on controller 2 controls the foundation grabber servos
        if(button_b && !active1) {
            active1 = true;
            if (iterations1 % 2 == 0) {
                servoFoundation1.setPosition(-1.0);
                servoFoundation2.setPosition(1.0);
                iterations1++;
            } else {
                servoFoundation1.setPosition(1.0);
                servoFoundation2.setPosition(-1.0);
                iterations1++;
            }

        }
        else if (!button_b) {
            active1 = false;
        }

        // dpad down button on controller 2 controls dropping the capstone
        if(button_dpad_down && !active2) {
            active2 = true;
            if (iterations2 % 2 == 0) {
                servoCapstone.setPosition(1.0);
                iterations2++;
            } else {
                servoCapstone.setPosition(-1.0);
                iterations2++;
            }

        }
        else if (!button_dpad_down) {
            active2 = false;
        }

        // The right bumper button toggles slowmode.  This simulates a manual transmission.
        if (button_rb && !button_rb_IsActive){
            button_rb_IsActive = true;
            slowmode = !slowmode;
        }
        else if (!button_rb) {
            button_rb_IsActive = false;
        }

        // If our simulated manual transmission is in slowmode we divide the joystick values
        // by 3 to simulate a slower gear ratio on the robot.
        if (slowmode){
            left_y = left_y / 3;
            left_x = left_x / 3;
            right_x = right_x / 3;
        }

        // left_y controls forward and backward power
        // left_x controls the strafing power
        // right_x is turning the robot
        // Combining the values together makes the robot go in any direction and rotate.
        leftFrontPower   = Range.clip(left_y + right_x + left_x, -1.0, 1.0) ;
        rightFrontPower  = Range.clip(left_y - right_x - left_x, -1.0, 1.0) ;
        leftRearPower    = Range.clip(left_y + right_x - left_x, -1.0, 1.0) ;
        rightRearPower   = Range.clip(left_y - right_x + left_x, -1.0, 1.0) ;

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);

        // Calculate the number of times this method (loop()) get called per second
        double currentTime = runtime.milliseconds();
        loops += 1;
        if ((currentTime - previousTime) >= 1000){
            loopsPerSecond = loops / ((currentTime - previousTime) / 1000.0);
            previousTime = currentTime;
            loops = 0;
        }

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f), leftRear (%.2f), rightRear (%.2f)", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
        telemetry.addData("Heading", "%.1f", AngleUnit.DEGREES.normalize(angles.firstAngle));
        telemetry.addData("Loop per Seconds", "%.2f lps", loopsPerSecond);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
