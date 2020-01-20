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
    private boolean positionOne = false;
    private boolean positionTwo = false;
    private boolean positionThree = false;
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
        block.init(hardwareMap, "block_servo0", -1.0, 1.0);
        grabber.init(hardwareMap, "grabber_servo4", 1.0, -1.0);
        foundation.init(hardwareMap, "foundation_servo1", "foundation_servo2", 1.0, -1.0);
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

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        //waiting for start
        telemetry.addData("Mode", "waiting for start");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        driveTrain.gyroStrafeToBlock(this, runtime, distanceSensor, .9, 10, DriveTrain.STRAFE_LEFT, 0,5 );
        driveTrain.gyroStrafeToBlock(this, runtime, distanceSensor, .2, 2.8, DriveTrain.STRAFE_LEFT, 0,10 );
        driveTrain.encoderDriveDistance(this, runtime, distanceSensor,.4, 10, 0,5);
        driveTrain.gyroDrive(this, runtime, -0.20, -5.5, 0, 10);

        if(colorDistance.isStone()) {
            driveTrain.gyroDrive(this, runtime, -0.20, -8, 0, 10);
            positionTwo = true;
            if(colorDistance.isStone()) {
                driveTrain.gyroDrive(this, runtime, -0.20, -8, 0, 10);
                positionThree = true;
            }
        } else {
            positionOne = true;
        }

        if(positionTwo == true) {
            offset = 1;
        } if(positionThree == true) {
            offset = 2;
        } else {
            offset = 0;
        }

        block.down();
        sleep(750);
        grabber.down();
        sleep(500);
        block.up();
        sleep(500);
        driveTrain.gyroDrive(this, runtime, 1, 68 + (offset*8), 0, 10);
        block.down();
        sleep(750);
        grabber.up();
        sleep(500);
        block.up();
        sleep(500);
        driveTrain.encoderStafe(this,runtime, .75,4, false, 5);
        driveTrain.gyroDrive(this, runtime, -1,-94 - (offset*8),0,7);
        sleep(200);
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        if ((distance > 3) && (distance < 10)){
            driveTrain.gyroStrafeToBlock(this, runtime, distanceSensor, 0.2, 1.5, true, 0,8 );
        } else {
            //driveTrain.gyroStrafe(this, runtime, distanceSensor, .2, 1, false, 90,5 );
        }
        block.down();
        sleep(750);
        grabber.down();
        sleep(500);
        block.up();
        sleep(200);
        driveTrain.gyroDrive(this, runtime, 1,103 + (offset*8),0,7);
        block.down();
        sleep(500);
        grabber.up();
        sleep(500);
        block.up();
        sleep(500);
        driveTrain.encoderStafe(this,runtime, .75,6, false, 3);
        driveTrain.gyroDrive(this, runtime, -1,-45,0,7);

        /*
        driveTrain.encoderStafe(this,runtime, .75,6, false, 3);
        driveTrain.rotate(this, 80,.75);
        driveTrain.gyroDrive(this, runtime, 0.50, 9, 90, 8);
        foundation.down();
        sleep(500);
        driveTrain.gyroDrive(this, runtime, -1,-25,90,7);
        driveTrain.rotate(this,0,.85);
        foundation.up();
        driveTrain.gyroDrive(this, runtime, -1,-50,0,7);

        telemetry.addData("Path", "Complete");
        telemetry.update();

         */
    }
}