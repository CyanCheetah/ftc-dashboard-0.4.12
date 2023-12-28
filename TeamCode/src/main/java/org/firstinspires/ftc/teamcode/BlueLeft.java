/**
 * @author CyanCheeah
 * This is the BlueLeft autonomous that scores on yellow as well.
 */
package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;
@Autonomous
public class BlueLeft extends LinearOpMode
{
    /**
     * swing ser os
     */
    private static Servo servoOne = null;
    private static Servo servoTwo = null;
    /**
     * arm for pixel
     */
    private static Servo Turn = null;
    OpenCvWebcam webcam;
    /**
     * 4 drive motors
     */
    private static DcMotor frontl = null;
    private static DcMotor frontr = null;
    private static DcMotor bottoml = null;
    private static DcMotor bottomr = null;
    /**
     * pipeline defined.
     */
    SkystoneDeterminationPipeline pipeline = new SkystoneDeterminationPipeline();
    /**
     * motor constant values.
     */
    MotorConstantValues constants = new MotorConstantValues();
    /**
     * elevator motors
     */
    private static DcMotor rightLift = null;
    private static DcMotor leftLift = null;
    /**
     * bucket servo
     */
    private static Servo Bucket = null;
    /**
     * swing servo
     */
    private static Servo Swing = null;
    @Override
    public void runOpMode()

    {
        /**
         * CameraMonitorViewID
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        /**
         * webcam used
         */
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cameraMonitorViewId"), cameraMonitorViewId);
        /**
         * sets webcam to that pipeline color library
         */
        webcam.setPipeline(pipeline);
        /**
         * for recording value in each pos
         */
        int first,second,third;
        /**
         * for calculating avg
         */
        int sum1 = 0, sum2 = 0, sum3 = 0;
        /**
         * counter for averages
         */
        int counter = 0;
        /**
         * true if not ran false if ran
         */
        boolean ran = true;
        /**
         * constants for the servos.
         */
        double clawFullOpen = constants.getClawFullOpen();
        double swingPos = constants.getSwingOutPosition();
        double bucketPos = constants.getBucketOutPosition();
        double bucketInPos = constants.getBucketInPosition();
        /**
         * declaring motor names
         */
        leftLift = hardwareMap.get(DcMotor.class,"leftLift");
        rightLift = hardwareMap.get(DcMotor.class,"rightLift");
        Bucket = hardwareMap.get(Servo.class, "Bucket");
        servoOne = hardwareMap.servo.get("servoOne");
        servoTwo = hardwareMap.servo.get("servoTwo");
        Swing = hardwareMap.servo.get("Swing");
        Turn = hardwareMap.get(Servo.class, "Turn");
        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                //funny there isnt any error handling exceptions -cyan
            }
        });
        telemetry.addLine("Waiting for start");
        telemetry.update();

        //Setting Directions of motors.
        //leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Turn.setPosition(.95);
        sleep(3000);
        while (!opModeIsActive())  {
            sleep(1000);
            first = pipeline.isPos1();
            second = pipeline.isPos2();
            third = pipeline.isPos3();
            telemetry.addData("1", first);
            telemetry.addData("2", second);
            telemetry.addData("3", third);
            telemetry.update();
            sum1 += first;
            sum2 += second;
            sum3 += third;
            counter++;

        }
        waitForStart();
        if (opModeIsActive())
        {
            //redundancy
            if (sum1 == 0 || sum2 == 0 || sum3 == 0)  {
                sleep(1000);
                first = pipeline.isPos1();
                second = pipeline.isPos2();
                third = pipeline.isPos3();
                telemetry.addData("1", first);
                telemetry.addData("2", second);
                telemetry.addData("3", third);
                telemetry.update();
                for (int i = 0; i < 4; i++) {
                    first = pipeline.isPos1();
                    second = pipeline.isPos2();
                    third = pipeline.isPos3();
                    sum1 += first;
                    sum2 += second;
                    sum3 += third;
                    sleep(500);
                }
            }
            //calculating avg's
            first = sum1 / counter;
            second = sum2 / counter;
            third = sum3 / counter;
            int maxOneTwo = Math.max(first, second);
            int max = Math.max(maxOneTwo, third);
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            //multiply this number by the inches needed to travel: 0.68571429
            if (ran) {
                if (max == first) {
                    telemetry.addData("1", first);
                    Bucket.setPosition(bucketInPos + .05);
                    Trajectory trajectoryFirst0 = drive.trajectoryBuilder(new Pose2d())
                            .lineToLinearHeading(new Pose2d(27.5, 5, Math.toRadians(71)))
                            .build();
                    Trajectory trajectoryFirst1 = drive.trajectoryBuilder(new Pose2d())
                            .forward(4)
                            .addTemporalMarker(3, () -> {
                                Turn.setPosition(clawFullOpen);
                            })
                            .build();
                    Trajectory trajectoryFirst2 = drive.trajectoryBuilder(new Pose2d())
                            .strafeRight(25)
                            .build();
                    Trajectory trajectoryFirst3 = drive.trajectoryBuilder(new Pose2d())
                            .forward(8)
                            .build();
                    Trajectory trajectoryFirst5 = drive.trajectoryBuilder(new Pose2d())
                            .forward(9)
                            .build();
                    Trajectory trajectoryFirst4 = drive.trajectoryBuilder(new Pose2d())
                            .strafeLeft(31)
                            .build();
                    Trajectory back1 = drive.trajectoryBuilder(new Pose2d())
                            .back(3)
                            .build();
                    Trajectory firt9 = drive.trajectoryBuilder(new Pose2d())
                            .back(4)
                            .build();
                    Trajectory first10 = drive.trajectoryBuilder(new Pose2d())
                            .strafeLeft(21)
                            .build();
                    drive.followTrajectory(trajectoryFirst0);
                    drive.followTrajectory(trajectoryFirst1);
                    drive.followTrajectory(trajectoryFirst5);
                    drive.followTrajectory(trajectoryFirst2);
                    drive.followTrajectory(trajectoryFirst3);
                    drive.followTrajectory(trajectoryFirst4);
                    drive.followTrajectory(trajectoryFirst5);
                    drive.followTrajectory(back1);
                    leftLift.setPower(.3);
                    rightLift.setPower(-.3);
                    sleep(2100);
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    Swing.setPosition(swingPos);
                    sleep(2000);
                    Bucket.setPosition(bucketPos);
                    sleep(400);
                    leftLift.setPower(.3);
                    rightLift.setPower(-.3);
                    sleep(600);
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    drive.followTrajectory(firt9);
                    drive.followTrajectory(first10);

                } else if (max == second) {
                    telemetry.addData("2", second);
                    Bucket.setPosition(bucketInPos + .05);
                    Trajectory trajectoryMiddle0 = drive.trajectoryBuilder(new Pose2d())
                            .forward(31.3)
                            .addTemporalMarker(4, () -> {
                                Turn.setPosition(clawFullOpen);
                            })
                            .build();
                    Trajectory trajectoryMiddle1 = drive.trajectoryBuilder(new Pose2d())
                            .forward(3)
                            .build();
                    Trajectory trajectoryMiddle2 = drive.trajectoryBuilder(new Pose2d())
                            .lineToLinearHeading(new Pose2d(0, 25, Math.toRadians(77)))
                            .build();
                    Trajectory trajectoryMiddle3 = drive.trajectoryBuilder(new Pose2d())
                            .strafeLeft(24)
                            .build();
                    Trajectory trajectoryMiddle4 = drive.trajectoryBuilder(new Pose2d())
                            .forward(17)
                            .build();
                    Trajectory trajectoryMiddle5 = drive.trajectoryBuilder(new Pose2d())
                            .back(4)
                            .build();
                    Trajectory trajectoryMiddle6 = drive.trajectoryBuilder(new Pose2d())
                            .strafeLeft(30)
                            .build();
                    Trajectory back2 = drive.trajectoryBuilder(new Pose2d())
                            .back(3)
                            .build();
                    drive.followTrajectory(trajectoryMiddle0);
                    drive.followTrajectory(trajectoryMiddle1);
                    drive.followTrajectory(trajectoryMiddle2);
                    drive.followTrajectory(trajectoryMiddle3);
                    drive.followTrajectory(trajectoryMiddle4);
                    drive.followTrajectory(back2);
                    leftLift.setPower(.3);
                    rightLift.setPower(-.3);
                    sleep(1900);
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    Swing.setPosition(swingPos);
                    sleep(2000);
                    Bucket.setPosition(bucketPos);
                    sleep(400);
                    leftLift.setPower(.3);
                    rightLift.setPower(-.3);
                    sleep(600);
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    drive.followTrajectory(trajectoryMiddle5);
                    drive.followTrajectory(trajectoryMiddle6);
                    // drive.followTrajectory(trajectoryMiddle5);
                    ran = false;


                } else if (max == third) {
                    telemetry.addData("3", third);
                    Bucket.setPosition(bucketInPos + .05);
                    Trajectory trajectoryRight0 = drive.trajectoryBuilder(new Pose2d())
                            .lineToLinearHeading(new Pose2d(26, -5, Math.toRadians(71)))
                            .build();
                    Trajectory trajectoryRight1 = drive.trajectoryBuilder(new Pose2d())
                            .back(7)
                            .addTemporalMarker(4, () -> {
                                Turn.setPosition(clawFullOpen);
                            })
                            .build();
                    Trajectory trajectoryRight2 = drive.trajectoryBuilder(new Pose2d())
                            .forward(32)
                            .build();
                    Trajectory trajectoryRight3 = drive.trajectoryBuilder(new Pose2d())
                            .strafeRight(12)
                            .build();
                    Trajectory trajectoryRight10 = drive.trajectoryBuilder(new Pose2d())
                            .forward(5)
                            .build();
                    Trajectory trajectoryRight4 = drive.trajectoryBuilder(new Pose2d())
                            .back(5)
                            .build();
                    Trajectory trajectoryRight5 = drive.trajectoryBuilder(new Pose2d())
                            .strafeLeft(41)
                            .build();
                    Trajectory back3 = drive.trajectoryBuilder(new Pose2d())
                            .back(4)
                            .build();
                    drive.followTrajectory(trajectoryRight0);
                    drive.followTrajectory(trajectoryRight1);
                    drive.followTrajectory(trajectoryRight2);
                    drive.followTrajectory(trajectoryRight3);
                    drive.followTrajectory(trajectoryRight10);
                    drive.followTrajectory(back3);
                    leftLift.setPower(.3);
                    rightLift.setPower(-.3);
                    sleep(2000);
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    Swing.setPosition(swingPos);
                    sleep(2000);
                    Bucket.setPosition(bucketPos);
                    sleep(400);
                    leftLift.setPower(.3);
                    rightLift.setPower(-.3);
                    sleep(600);
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    drive.followTrajectory(trajectoryRight4);
                    drive.followTrajectory(trajectoryRight5);
                    ran = false;
                }
            }
        }
    }
}