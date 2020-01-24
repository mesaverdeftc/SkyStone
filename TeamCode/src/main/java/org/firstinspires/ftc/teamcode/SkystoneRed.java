package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="SkystoneRed", group="Linear Opmode")
//@Disabled
public class SkystoneRed extends LinearOpMode {

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

        driveTrain.gyroStrafeToBlock(this, runtime, distanceSensor, .5, 10, DriveTrain.STRAFE_LEFT, 0,5 );
        driveTrain.gyroStrafeToBlock(this, runtime, distanceSensor, .1, 2.8, DriveTrain.STRAFE_LEFT, 0,10 );
        if(!colorDistance.isStone()) {
            driveTrain.gyroStrafeToBlock(this, runtime, distanceSensor, .1, 1.8, DriveTrain.STRAFE_LEFT, 0,10 );
        }
        driveTrain.gyroDriveBlockEdge(this, runtime, distanceSensor,.3, 10, 0,5);
        driveTrain.gyroDrive(this, runtime, -0.4, -5.5, 0, 10);

        if(colorDistance.isStone()) {
            driveTrain.gyroDrive(this, runtime, -0.4, -8, 0, 10);
            offset = 1;
            if(colorDistance.isStone()) {
                driveTrain.gyroDrive(this, runtime, -0.4, -8, 0, 10);
                offset = 2;

            }
        }

        block.down();
        sleep(600);
        grabber.down();
        sleep(500);
        block.up();
        sleep(500);
        driveTrain.gyroDrive(this, runtime, .4, 70 + (offset*8.8), 0, 10);
        block.down();
        sleep(750);
        grabber.up();
        sleep(200);
        block.up();
        sleep(200);
        // driveTrain.encoderStafe(this,runtime, .75,.3, false, 5);
        driveTrain.gyroDrive(this, runtime, -.4,-94 - (offset*8.6),0,7);
        sleep(200);
/*
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        if ((distance > 4.3) && (distance < 10)){
            driveTrain.gyroStrafeToBlock(this, runtime, distanceSensor, 0.2, .2, true, 0,8 );
        } else {
            //driveTrain.gyroStrafe(this, runtime, distanceSensor, .2, 1, false, 90,5 );
        }
 */
        block.down();
        sleep(750);
        grabber.down();
        sleep(500);
        block.up();
        sleep(200);
        driveTrain.gyroDrive(this, runtime, .4,106 + (offset*8.6),0,7);
        block.down();
        sleep(500);
        grabber.up();
        sleep(500);
        block.up();
        sleep(500);

        // driveTrain.gyroDrive(this, runtime, -1,-45,0,7);


        driveTrain.encoderStafe(this,runtime, .5,6, false, 3);
        driveTrain.rotate(this, 90,.4);
        driveTrain.gyroDrive(this, runtime, 0.4, 11, 90, 8);
        foundation.down();
        sleep(500);
        driveTrain.gyroDrive(this, runtime, -.4, -41.3, 90, 8);
        foundation.up();
        driveTrain.encoderStrafeOffsetUp(this, runtime, .4, 37, DriveTrain.STRAFE_LEFT, 10);
        /*
        driveTrain.gyroDrive(this, runtime, -1,-25,90,7);
        driveTrain.rotate(this,0,.75);
        foundation.up();
        driveTrain.gyroDrive(this, runtime, -1,-50,0,7);
        */
        telemetry.addData("Path", "Complete");
        telemetry.update();


    }
}