package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TestMotors", group="Linear Opmode")
//@Disabled
public class TestMotors extends LinearOpMode{

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain = new DriveTrain();
    private DistanceSensor distanceSensor = null;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        driveTrain.init(hardwareMap);
        distanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "distance_1");

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !driveTrain.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status", driveTrain.imu.getCalibrationStatus().toString());

        //waiting for start
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

       /* sleep(1000);
        driveTrain.gyroDrive(this, runtime, 0.5, 8, 0, 30);
        sleep(5000);
        driveTrain.gyroDrive(this, runtime, -0.5, -8, 0, 30);
        sleep(1000);
        telemetry.addData("Path", "Complete");
        telemetry.update();


        */

        sleep(1000);
        driveTrain.gyroStrafeToBlock(this, runtime, distanceSensor, 0.7, 12, 0, 30);
        driveTrain.gyroStrafeToBlock(this, runtime, distanceSensor, 0.2, 2.75, 0, 30);
        sleep(5000);

    }
}