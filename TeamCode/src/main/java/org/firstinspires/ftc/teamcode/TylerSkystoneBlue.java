package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="TylerSkystoneBlue", group="Linear Opmode")
@Disabled
public class TylerSkystoneBlue extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain = new DriveTrain();
    private Foundation foundation = new Foundation();
    private Attachment block = new Attachment();
    private Attachment grabber = new Attachment();
    private DistanceSensor distanceSensor = null;
    private ColorDistance colorSensor = new ColorDistance();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        driveTrain.init(hardwareMap);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_1");
        block.init(hardwareMap, "block_servo0", 0, 1.0);
        grabber.init(hardwareMap, "grabber_servo4", 1.0, 0);
        foundation.init(hardwareMap, "foundation_servo1", "foundation_servo2", 1.0, 0);
        foundation.up();
        block.up();
        grabber.up();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !driveTrain.imu.isGyroCalibrated())
        {
            sleep(500);
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

        driveTrain.gyroStrafeToBlock(this, runtime, distanceSensor, .9, 10, 90,5 );
        driveTrain.gyroStrafeToBlock(this, runtime, distanceSensor, .2, 2.75, 90,10 );
        block.down();
        sleep(750);
        grabber.down();
        sleep(500);
        block.up();
        sleep(200);
        driveTrain.gyroDrive(this, runtime, 1, -68, 0, 10);
        block.down();
        sleep(750);
        grabber.up();
        sleep(500);
        block.up();
        sleep(500);
        driveTrain.encoderStafe(this,runtime, .75,4, false, 5);
        driveTrain.gyroDrive(this, runtime, 1,86,0,7);
        sleep(200);
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        if ((distance > 3) && (distance < 12)){
            driveTrain.gyroStrafeToBlock(this, runtime, distanceSensor, 0.2, 2.75, 90,8 );
        } else {
            //driveTrain.gyroStrafe(this, runtime, distanceSensor, .2, 1, false, 90,5 );
        }
        block.down();
        sleep(750);
        grabber.down();
        sleep(500);
        block.up();
        sleep(200);
        driveTrain.gyroDrive(this, runtime, 1,-94,0,7);
        block.down();
        sleep(500);
        grabber.up();
        sleep(500);
        block.up();
        sleep(500);
        driveTrain.encoderStafe(this,runtime, .75,6, true, 3);
        driveTrain.rotate(this, 80,.75);
        // driveTrain.encoderStafe(this,runtime, 0.5,27, true, 5);
        driveTrain.gyroDrive(this, runtime, 0.50, 11, 90, 8);
        foundation.down();
        sleep(500);
        driveTrain.gyroDrive(this, runtime, -1, -32, 0,5);
        // driveTrain.gyroDrive(this, runtime, -.80, 4, 90, 4);
        foundation.up();
        driveTrain.encoderStafe(this, runtime, 1, 65, DriveTrain.STRAFE_RIGHT, 10 );
        sleep(1000000);
        //driveTrain.gyroDrive(this, runtime, 1, -55, 0, 6);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
   }