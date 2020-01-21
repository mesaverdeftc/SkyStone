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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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

@TeleOp(name="Teleop", group="Iterative Opmode")
//@Disabled
public class Teleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain = new DriveTrain();
    private Attachment block = new Attachment();
    private Attachment grabber = new Attachment();
    private Foundation foundation = new Foundation();
    private Attachment capstone = new Attachment();
    private ColorDistance colorDistance = new ColorDistance();
    private DistanceSensor distanceSensor = null;

    private ButtonToggle buttonY = new ButtonToggle();
    private ButtonToggle buttonA = new ButtonToggle();
    private ButtonToggle buttonB = new ButtonToggle();
    private ButtonToggle button_rb = new ButtonToggle();
    private ButtonToggle button_lb = new ButtonToggle();
    private ButtonToggle button_dpad_down = new ButtonToggle();

    private boolean slowmode = false;
    private boolean fieldCentric = false;

    private double previousTime = 0.0;
    private double loopsPerSecond = 0.0;
    private double loops = 0.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        driveTrain.init(hardwareMap);
        block.init(hardwareMap, "block_servo0", -1.0, 1.0);
        grabber.init(hardwareMap, "grabber_servo4", 1.0, -1.0);
        foundation.init(hardwareMap, "foundation_servo1", "foundation_servo2", 1.0, -1.0);
        capstone.init(hardwareMap, "capstone_servo3", 1.0, -1.0);
        colorDistance.init(hardwareMap, "block_color_2");
        distanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "distance_1");

        // Tell the driver that initialization is complete.
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

        double left_x = gamepad1.left_stick_x;
        double left_y = -gamepad1.left_stick_y;
        double right_x = gamepad1.right_stick_x;

        // Check is we need to toggle the block attachment's position
        if (buttonY.toggled(gamepad2.y))
            block.toggle(buttonY.toggleState);

        // Check is we need to toggle the grabber attachment's position
        if (buttonA.toggled(gamepad2.a))
            grabber.toggle(buttonA.toggleState);

        // Check is we need to toggle the foundation attachment's position
        if (buttonB.toggled(gamepad2.b))
            foundation.toggle(buttonB.toggleState);

        // Check is we need to toggle the capstone attachment's position
        if (button_dpad_down.toggled(gamepad2.dpad_down))
            capstone.toggle(button_dpad_down.toggleState);

        // The right bumper button toggles slowmode.  This simulates a manual transmission.
        if (button_rb.toggled(gamepad1.right_bumper))
            slowmode = !slowmode;

        // The left bumper button toggles robot centric and field centric driving.
        if (button_lb.toggled(gamepad1.left_bumper))
            fieldCentric = !fieldCentric;

        // The dpad_up button resets the field centric 0 angle
        if(gamepad1.dpad_up) {
            driveTrain.resetAngle();
        }

        driveTrain.drive(left_x, left_y, right_x, fieldCentric, slowmode);

        // Calculate the number of times this method (loop()) get called per second
        double currentTime = runtime.milliseconds();
        loops += 1;
        if ((currentTime - previousTime) >= 1000){
            loopsPerSecond = loops / ((currentTime - previousTime) / 1000.0);
            previousTime = currentTime;
            loops = 0;
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Loop per Seconds", "%.2f lps", loopsPerSecond);
        telemetry.addData("Heading", "%.1f", driveTrain.getHeading());
        telemetry.addData("Stick Values", "leftX = %.2f, leftY = %.2f", left_x, left_y);
        telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f), leftRear (%.2f), rightRear (%.2f)",
                driveTrain.leftFrontPower, driveTrain.rightFrontPower, driveTrain.leftRearPower, driveTrain.rightRearPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        driveTrain.stop();
    }
}