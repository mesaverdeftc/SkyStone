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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="Hudson Teleop", group="Iterative Opmode")
// @Disabled
public class HudsonsTeleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain = new DriveTrain();

    private Attachment block = new Attachment();
    private Attachment grabber = new Attachment();
    private Foundation foundation = new Foundation();
    private Attachment capstone = new Attachment();
    private DcMotor tapeMeasure = null;
    private DcMotor pincherLeft = null;
    private DcMotor pincherRight = null;

    private ButtonToggle buttonY = new ButtonToggle();
    private ButtonToggle buttonA = new ButtonToggle();
    private ButtonToggle buttonB = new ButtonToggle();
    private ButtonToggle button_rb = new ButtonToggle();
    private ButtonToggle button_lb = new ButtonToggle();
    private ButtonToggle button_dpad_down = new ButtonToggle();
    private ButtonToggle button_rb_gamepad2 = new ButtonToggle();

    private DistanceSensor distanceSensor = null;

    private boolean slowmode = false;
    private boolean fieldCentric = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        block.init(hardwareMap, "block_servo0", 0, 1.0);
        foundation.init(hardwareMap, "foundation_servo1", "foundation_servo2",1.0, 0);
        capstone.init(hardwareMap, "capstone_servo3", 0, 1.0);
        grabber.init(hardwareMap, "grabber_servo4", 0, 1.0);
        distanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "distance_1");

        tapeMeasure = hardwareMap.get(DcMotor.class, "tape_measure_drive");
        tapeMeasure.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tapeMeasure.setDirection(DcMotor.Direction.FORWARD);

        pincherLeft = hardwareMap.get(DcMotor.class, "pincher_left_0");
        pincherRight = hardwareMap.get(DcMotor.class, "pincher_right_1");
        pincherLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pincherRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pincherLeft.setDirection(DcMotor.Direction.FORWARD);
        pincherRight.setDirection(DcMotor.Direction.REVERSE);

        driveTrain.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {}

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left_x = gamepad1.left_stick_x;
        double left_y = -gamepad1.left_stick_y;
        double right_x = gamepad1.right_stick_x;

        double tape_power = -gamepad2.right_stick_y;
        tapeMeasure.setPower(tape_power);

        if(button_rb_gamepad2.toggled(gamepad2.right_bumper)){

            if(button_rb_gamepad2.toggleState){
               pincherLeft.setPower(0.5);
               pincherRight.setPower(0.5);
            } else{
                pincherLeft.setPower(-0.375);
                pincherRight.setPower(-0.5);
            }
        }
        if(button_lb.toggled(gamepad2.left_bumper)){
            pincherLeft.setPower(0);
            pincherRight.setPower(0);
        }

        if(buttonA.toggled(gamepad1.a)) {
            driveTrain.gyroStrafeAwayBlock(runtime, distanceSensor, .4, 1.5, 5);
            block.setPosition(-1.0);
            grabber.setPosition(1.0);
            block.setPosition(1.0);
        }

        /*if(buttonY.toggled(gamepad2.y)) {
            block.toggle(buttonY.toggleState);
        }*/

        block.setPosition(gamepad2.left_stick_y);

        if(buttonA.toggled(gamepad2.a)) {
            grabber.toggle(buttonA.toggleState);
        }

        if(buttonB.toggled(gamepad2.b)) {
            foundation.toggle(buttonB.toggleState);
        }

        if(button_dpad_down.toggled(gamepad2.dpad_down)) {
            capstone.toggle(button_dpad_down.toggleState);
        }

        if(gamepad1.dpad_up) {
            driveTrain.resetAngle();
        }

        if(button_lb.toggled(gamepad1.left_bumper)) {

            fieldCentric = !fieldCentric;
        }

        if(button_rb.toggled(gamepad1.right_bumper)) {
            slowmode = !slowmode;
        }

        driveTrain.drive(left_x,left_y, right_x, fieldCentric, slowmode);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Values", "leftX = %.2f, leftY = %.2f", left_x, left_y);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f), leftRear (%.2f), rightRear (%.2f)",
                driveTrain.leftFrontPower, driveTrain.rightFrontPower, driveTrain.leftRearPower, driveTrain.rightRearPower);
        telemetry.addData("Heading", "%.1f", driveTrain.getHeading());
    }

    @Override
    public void stop() {}
}



