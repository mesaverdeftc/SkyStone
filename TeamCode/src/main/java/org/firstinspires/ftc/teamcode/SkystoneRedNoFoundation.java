package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="SkystoneRedNoFoundation", group="Linear Opmode")
//@Disabled
public class SkystoneRedNoFoundation extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain = new DriveTrain();
    private Foundation foundation = new Foundation();
    private Attachment block = new Attachment();
    private Attachment grabber = new Attachment();
    private DistanceSensor distanceSensor = null;
    private ColorDistance colorDistance = new ColorDistance();
    private int offset = 0;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        driveTrain.init(hardwareMap);

        colorDistance.init(hardwareMap, "block_color_2");
        distanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "distance_1");
        block.init(hardwareMap, "block_servo0", 0, 1.0);
        grabber.init(hardwareMap, "grabber_servo4", 1.0, 0);
        foundation.init(hardwareMap, "foundation_servo1", "foundation_servo2", 1.0, 0);
        foundation.up();
        block.up();
        grabber.up();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !driveTrain.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status", driveTrain.imu.getCalibrationStatus().toString());
        telemetry.update();

        //waiting for start
        telemetry.addData("Mode", "waiting for start");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        driveTrain.gyroStrafeToBlock(this, runtime, distanceSensor, .7, 12, 0,20 );
        driveTrain.gyroStrafeToBlock(this, runtime, distanceSensor, .2, 2.6, 0,16 );
        if(!colorDistance.isStone()) {
            driveTrain.gyroStrafeToBlock(this, runtime, distanceSensor, .2, 1.4, 0,10 );
        }
        driveTrain.gyroDriveBlockEdge(this, runtime, distanceSensor, .3, 10, 0,5);
        driveTrain.gyroDrive(this, runtime, -0.3, -5.5, 0, 10);

        if(colorDistance.isStone()) {
            driveTrain.gyroDrive(this, runtime, -0.3, -8, 0, 10);
            offset = 1;
            if(colorDistance.isStone()) {
                driveTrain.gyroDrive(this, runtime, -0.3, -8, 0, 10);
                offset = 2;
            }
        }

        block.down();
        sleep(600);
        grabber.down();
        sleep(500);
        block.up();
        sleep(500);
        //driveTrain.encoderStafe(this,runtime, .5,4, driveTrain.STRAFE_RIGHT, 3);
        driveTrain.gyroDrive(this, runtime, 1.0, 42, 0, 10);
        //driveTrain.encoderStafe(this,runtime, .5,8, driveTrain.STRAFE_LEFT, 3);
        block.down();
        sleep(700);
        grabber.up();
        sleep(200);
        block.up();
        sleep(200);
        //driveTrain.encoderStafe(this,runtime, .5,8, driveTrain.STRAFE_RIGHT, 5);
        if (offset == 2) offset = -2;
        driveTrain.gyroDrive(this, runtime, -1.0, -50 -  (offset*8),0,7);
        sleep(200);


        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        if ((distance > 4.3) && (distance < 10)){
            driveTrain.gyroStrafeToBlock(this, runtime, distanceSensor, 0.2, 1.5, 0,8 );

        }
        driveTrain.encoderStafe(this,runtime, .2,.2, driveTrain.STRAFE_RIGHT, 3);



        block.down();
        sleep(750);
        grabber.down();
        sleep(400);
        block.up();
        sleep(200);
        driveTrain.gyroDrive(this, runtime, 1.0, 42 + (offset*8), 0, 10);
        block.down();
        sleep(750);
        grabber.up();
        sleep(400);
        block.up();
        sleep(200);

        if (offset == 0){
            driveTrain.gyroDrive(this, runtime, -1.0, -46, 0, 10);
            block.down();
            sleep(400);
            grabber.down();
            sleep(200);
            block.up();
            sleep(200);

            driveTrain.gyroDrive(this, runtime, 1.0, 46, 0, 10);
            block.down();
            sleep(400);
            grabber.up();
            sleep(200);
            block.up();

            driveTrain.gyroDrive(this, runtime, -1.0, -54, 0, 10);
            block.down();
            sleep(400);
            grabber.down();
            sleep(200);
            block.up();
            sleep(200);

            driveTrain.gyroDrive(this, runtime, 1.0, 52, 0, 10);
            block.down();
            sleep(400);
            grabber.up();
            sleep(200);
            block.up();
        }

        if (offset == 1) {
            driveTrain.gyroDrive(this, runtime, -1.0, -54, 0, 10);
            block.down();
            sleep(400);
            grabber.down();
            sleep(200);
            block.up();
            sleep(200);

            driveTrain.gyroDrive(this, runtime, 1.0, 54, 0, 10);
            block.down();
            sleep(400);
            grabber.up();
            sleep(200);
            block.up();

            driveTrain.gyroDrive(this, runtime, -1.0, -60, 0, 10);
            block.down();
            sleep(400);
            grabber.down();
            sleep(200);
            block.up();
            sleep(200);

            driveTrain.gyroDrive(this, runtime, 1.0, 60, 0, 10);
            block.down();
            sleep(400);
            grabber.up();
            sleep(200);
            block.up();
        }

        if (offset == 2) {
            driveTrain.gyroDrive(this, runtime, -1.0, -54, 0, 10);
            block.down();
            sleep(400);
            grabber.down();
            sleep(200);
            block.up();
            sleep(200);

            driveTrain.gyroDrive(this, runtime, 1.0, 54, 0, 10);
            block.down();
            sleep(400);
            grabber.up();
            sleep(200);
            block.up();

            driveTrain.gyroDrive(this, runtime, -1.0, -60, 0, 10);
            block.down();
            sleep(400);
            grabber.down();
            sleep(200);
            block.up();
            sleep(200);

            driveTrain.gyroDrive(this, runtime, 1.0, 60, 0, 10);
            block.down();
            sleep(400);
            grabber.up();
            sleep(200);
            block.up();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();


    }
}