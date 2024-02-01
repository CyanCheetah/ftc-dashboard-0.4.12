/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/**
 * @author CyanCheeah
 * This is the RedRight autonomous that scores on yellow as well.
 */
package org.firstinspires.ftc.teamcode;



// TODO: remove Actions from the core module?
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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.MotorConstantValues;
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
public class RedRight extends LinearOpMode
{
    private static Servo servoOne = null;

    private static Servo servoTwo = null;
    private static Servo Turn = null;


    OpenCvWebcam webcam;
    private static DcMotor frontl = null;
    private static DcMotor frontr = null;
    private static DcMotor bottoml = null;
    private static DcMotor bottomr = null;
    SkystoneDeterminationPipeline pipeline = new SkystoneDeterminationPipeline();
    private static DcMotor rightLift = null;
    private static DcMotor leftLift = null;
    private static Servo Bucket = null;
    @Override
    public void runOpMode()

    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cameraMonitorViewId"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);
        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */

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
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
        // leftLift = hardwareMap.get(DcMotor.class,"leftLift");
        // rightLift = hardwareMap.get(DcMotor.class,"rightLift");
        Servo servoOne = hardwareMap.servo.get("servoOne");
        Servo servoTwo = hardwareMap.servo.get("servoTwo");
        Servo Swing = hardwareMap.servo.get("Swing");

        //Bucket = hardwareMap.get(Servo.class, "Bucket");
        //Swing = hardwareMap.get(Servo.class, "Swing");
        //Setting Directions of motors.

        //leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*
         * Wait for the user to press start on the Driver Station
         */
        int first,second,third;
        Turn = hardwareMap.get(Servo.class, "Turn");
        Turn.setPosition(.95);
        while (pipeline.isPos1() == 0 && pipeline.isPos2() == 0 && pipeline.isPos3() == 0)  {

            sleep(1000);

            first = pipeline.isPos1();
            second = pipeline.isPos2();
            third = pipeline.isPos3();
            telemetry.addData("1", first);
            telemetry.addData("2", second);
            telemetry.addData("3", third);
            telemetry.update();

        }
        waitForStart();



        if (opModeIsActive())

        {

            int sum1 = 0, sum2 = 0, sum3 = 0;
            for (int i = 0; i < 4; i++) {
                first = pipeline.isPos1();
                second = pipeline.isPos2();
                third = pipeline.isPos3();
                sum1 += first;
                sum2 += second;
                sum3 += third;
                sleep(500);
            }
            first = sum1 / 4;
            second = sum2 / 4;
            third = sum3 / 4;
            telemetry.addData("1", first);
            telemetry.addData("2", second);
            telemetry.addData("3", third);
            leftLift = hardwareMap.get(DcMotor.class,"leftLift");
            rightLift = hardwareMap.get(DcMotor.class,"rightLift");
            Bucket = hardwareMap.get(Servo.class, "Bucket");
            int maxOneTwo = Math.min(first, second);
            int max = Math.min(maxOneTwo, third);
            boolean ran = true;
            MotorConstantValues constants = new MotorConstantValues();
            double clawFullOpen = 0;
            double swingPos = 0;
            double bucketPos = 0;
            double bucketInPos = 0;
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            //multiply this number by the inches needed to travel: 0.68571429
            if (ran) {
                if (max == first) {
                    telemetry.addData("1", first);
                    Bucket.setPosition(bucketInPos + .05);
                    Trajectory trajectoryFirst0 = drive.trajectoryBuilder(new Pose2d())
                            .forward(20)
                            .build();
                    TrajectorySequence ts = drive.trajectorySequenceBuilder(new Pose2d())
                            .turn(Math.toRadians(-102)) // Turns 45 degrees counter-clockwise
                            .build();
                    Trajectory trajectoryFirst2 = drive.trajectoryBuilder(new Pose2d())
                            .back(2)
                            .addTemporalMarker(3, () -> {
                                Turn.setPosition(clawFullOpen);
                            })
                            .build();
                    Trajectory trajectoryFirst3 = drive.trajectoryBuilder(new Pose2d())
                            .lineToConstantHeading(new Vector2d(31,16))
                            .build();
                    Trajectory park = drive.trajectoryBuilder(new Pose2d())
                            .strafeRight(37)
                            .build();
                    Trajectory park0 = drive.trajectoryBuilder(new Pose2d())
                            .back(3)
                            .build();
                    Trajectory back1 = drive.trajectoryBuilder(new Pose2d())
                            .back(2)
                            .build();

                    drive.followTrajectory(trajectoryFirst0);
                    drive.followTrajectorySequence(ts);
                    drive.followTrajectory(trajectoryFirst2);
                    sleep(1000);
                    drive.followTrajectory(trajectoryFirst3);

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
                    drive.followTrajectory(park0);
                    drive.followTrajectory(park);

                } else if (max == second) {
                    telemetry.addData("2", second);
                    Bucket.setPosition(bucketInPos + .05);
                    Trajectory trajectoryMiddle0 = drive.trajectoryBuilder(new Pose2d())
                            .forward(31.5)
                            .addTemporalMarker(4, () -> {
                                Turn.setPosition(clawFullOpen);
                            })
                            .build();
                    Trajectory trajectoryMiddle1 = drive.trajectoryBuilder(new Pose2d())
                            .forward(5)
                            .build();
                    Trajectory trajectoryMiddle2 = drive.trajectoryBuilder(new Pose2d())
                            .lineToLinearHeading(new Pose2d(0, -25, Math.toRadians(-77)))
                            .build();
                    Trajectory trajectoryMiddle3 = drive.trajectoryBuilder(new Pose2d())
                            .strafeRight(28)
                            .build();
                    Trajectory trajectoryMiddle4 = drive.trajectoryBuilder(new Pose2d())
                            .forward(15)
                            .build();
                    Trajectory trajectoryMiddle5 = drive.trajectoryBuilder(new Pose2d())
                            .back(5)
                            .build();
                    Trajectory trajectoryMiddle6 = drive.trajectoryBuilder(new Pose2d())
                            .strafeRight(30)
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
                    drive.followTrajectory(trajectoryMiddle5);
                    drive.followTrajectory(trajectoryMiddle6);
                    // drive.followTrajectory(trajectoryMiddle5);


                    ran = false;

                } else if (max == third) {
                    telemetry.addData("3", third);
                    Bucket.setPosition(bucketInPos + .05);
                    Trajectory trajectoryRight0 = drive.trajectoryBuilder(new Pose2d())
                            .lineToConstantHeading(new Vector2d(23, -22))
                            .build();
                    Trajectory rightStr = drive.trajectoryBuilder(new Pose2d())
                            .strafeRight(6)
                            .addTemporalMarker(1.5, () -> {
                                Turn.setPosition(clawFullOpen);
                            })
                            .build();
                    Trajectory trajectoryRight1 = drive.trajectoryBuilder(new Pose2d())
                            .lineToLinearHeading(new Pose2d(-30, -34.5, Math.toRadians(-77)))
                            .build();

                    Trajectory trajectoryRight2 = drive.trajectoryBuilder(new Pose2d())
                            .forward(13)
                            .build();
                    Trajectory rightpark = drive.trajectoryBuilder(new Pose2d())
                            .back(3)
                            .build();
                    Trajectory rightpark2 = drive.trajectoryBuilder(new Pose2d())
                            .strafeRight(37)
                            .build();
                    Trajectory back3 = drive.trajectoryBuilder(new Pose2d())
                            .back(3)
                            .build();
                    Trajectory back4 = drive.trajectoryBuilder(new Pose2d())
                            .forward(5)
                            .build();
                    drive.followTrajectory(trajectoryRight0);
                    drive.followTrajectory(rightStr);
                    drive.followTrajectory(trajectoryRight2);
                    drive.followTrajectory(trajectoryRight1);
                    drive.followTrajectory(back4);
                    drive.followTrajectory(back3);
                    sleep(1000);
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
                    drive.followTrajectory(rightpark);
                    drive.followTrajectory(rightpark2);
                    ran = false;
                }
            }
        }
    }
}