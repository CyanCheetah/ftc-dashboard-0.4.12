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
 * This is the RedLeft autonomous that scores only puruple.
 */
package org.firstinspires.ftc.teamcode;



// TODO: remove Actions from the core module?
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
public class RedLeftCycle extends LinearOpMode {
    private int region;
    private static CRServo IntakeUno = null;
    private static CRServo IntakeDos = null;
    private static CRServo IntakeRoller = null;
    private static Servo IntakePos = null;
    private static Servo OuttakeClaw = null;
    private static DcMotor rightLift = null;
    private static DcMotor leftLift = null;
    private static Servo OuttakeFlip = null;
    private static Servo OuttakeSpin = null;
    OpenCvWebcam webcam;
    private static DcMotor frontl = null;
    private static DcMotor frontr = null;
    private static DcMotor bottoml = null;
    private static DcMotor bottomr = null;
    RedSightPipeline pipeline;

    @Override
    public void runOpMode() {
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
        pipeline = new RedSightPipeline(telemetry);
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
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
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
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
        // leftLift = hardwareMap.get(DcMotor.class,"leftLift");
        // rightLift = hardwareMap.get(DcMotor.class,"rightLift");

        //Bucket = hardwareMap.get(Servo.class, "Bucket");
        //Swing = hardwareMap.get(Servo.class, "Swing");
        //Setting Directions of motors.

        //leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*
         * Wait for the user to press start on the Driver Station
         */
        boolean ran = true;
        IntakeUno = hardwareMap.get(CRServo.class, "IntakeUno");
        IntakeDos = hardwareMap.get(CRServo.class, "IntakeDos");
        IntakeRoller = hardwareMap.get(CRServo.class, "IntakeRoller");
        IntakePos = hardwareMap.get(Servo.class, "IntakePos");

        leftLift = hardwareMap.get(DcMotor.class,"leftLift");
        rightLift = hardwareMap.get(DcMotor.class,"rightLift");
        OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
        OuttakeFlip = hardwareMap.get(Servo.class, "OuttakeFlip");
        OuttakeSpin = hardwareMap.get(Servo.class, "OuttakeSpin");
        RedSightPipeline.SkystonePosition pos;
        while (!isStarted() && !isStopRequested()) {
            pos = pipeline.getAnalysis();
            telemetry.addData("Color", pos);
            telemetry.update();
        }
        pos = pipeline.getAnalysis();
        OuttakeClaw.setPosition(0.275);
        waitForStart();
        if (opModeIsActive()) {
            telemetry.addData("Color", pos);
            telemetry.update();
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            if (ran) {
                if (pos == RedSightPipeline.SkystonePosition.LEFT) {
                    telemetry.addData("1", pos);
                    TrajectorySequence Left = drive.trajectorySequenceBuilder(new Pose2d())
                            .lineToLinearHeading(new Pose2d(30, -3, Math.toRadians(-83)))
                            .back(9)
                            .addTemporalMarker(0, () -> {
                                IntakePos.setPosition(.913);
                            })
                            .addTemporalMarker(2.5, () -> {
                                IntakePos.setPosition(0.9);
                            })
                            .addTemporalMarker(2.3, () -> {
                                IntakePos.setPosition(0.89);
                                IntakeUno.setPower((.8));
                                IntakeDos.setPower((-.8));
                                IntakeRoller.setPower((-1));
                            })
                            .addTemporalMarker(6, () -> {
                                IntakePos.setPosition(0.913);
                                IntakeUno.setPower((0));
                                IntakeDos.setPower((0));
                                IntakeRoller.setPower((0));
                            })
                            .waitSeconds(1)
                            .lineToLinearHeading(new Pose2d(55, -38, Math.toRadians(-71)))
                            .addTemporalMarker(6, () -> {
                                sleep(300);
                                OuttakeFlip.setPosition(0.78);
                                sleep(300);
                                OuttakeSpin.setPosition(1-.185);
                            })
                            .forward(5)
                            .build();

                    Trajectory parkL = drive.trajectoryBuilder(new Pose2d())
                            .strafeRight(30)
                            .build();
                    Trajectory park2L = drive.trajectoryBuilder(new Pose2d())
                            .addTemporalMarker(2, () -> {
                                OuttakeSpin.setPosition(0.489);
                                sleep(200);
                                OuttakeFlip.setPosition(0.245);
                            })
                            .back(3)
                            .build();

                    drive.followTrajectorySequence(Left);
                    sleep(200);
                    OuttakeClaw.setPosition(0.275);
                    sleep(200);

                    drive.followTrajectory(park2L);
                    drive.followTrajectory(parkL);




                    ran = false;
                } else if (pos == RedSightPipeline.SkystonePosition.CENTER) {
                    telemetry.addData("2", pos);
                    TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                            .lineToLinearHeading(new Pose2d(29, -3, Math.toRadians(160)))
                            .back(3)
                            .forward(6)
                            .addTemporalMarker(0, () -> {
                                IntakePos.setPosition(.88);
                            })
                            .strafeRight(10)
                            .lineToLinearHeading(new Pose2d(60, 0, Math.toRadians(-125)))
                            .back(10)
                            .waitSeconds(0.5)
                            .forward(75)
                            //.lineToLinearHeading(new Pose2d(-47, -40, Math.toRadians(-71)))
                            //.lineToLinearHeading(new Pose2d(-22, -50, Math.toRadians(-71)))
                            .addTemporalMarker(2, () -> {
                                IntakeUno.setPower((.8));
                                IntakeDos.setPower((-.8));
                                IntakeRoller.setPower(-1);
                            })
                            .addTemporalMarker(3, () -> {
                                IntakeUno.setPower((0));
                                IntakeDos.setPower((0));
                                IntakeRoller.setPower(0);
                            })
                            .addTemporalMarker(4, () -> {
                                IntakePos.setPosition(.92);
                                IntakeUno.setPower((-.8));
                                IntakeDos.setPower((.8));
                                IntakeRoller.setPower(1);
                            })
                            .addTemporalMarker(7.5, () -> {
                                IntakeUno.setPower((0));
                                IntakeDos.setPower((0));
                                IntakeRoller.setPower((0));
                            })
                            .addTemporalMarker(9, () -> {
                                IntakeUno.setPower((-.8));
                                IntakeDos.setPower((.8));
                                IntakeRoller.setPower((1));
                            })
                            .addTemporalMarker(12, () -> {
                                IntakeUno.setPower((0));
                                IntakeDos.setPower((0));
                                IntakeRoller.setPower((0));
                            })
                            .addTemporalMarker(13, () -> {
                                OuttakeClaw.setPosition(0);
                            })
                            .forward(7)
                            .build();
                    Trajectory park = drive.trajectoryBuilder(new Pose2d())
                            .strafeLeft(30)
                            .build();
                    Trajectory park2 = drive.trajectoryBuilder(new Pose2d())
                            .back(3)
                            .build();

                    drive.followTrajectorySequence(trajSeq);
                    OuttakeClaw.setPosition(0);
                    rightLift.setPower((-.65));
                    leftLift.setPower((.65));
                    sleep(200);
                    rightLift.setPower((0));
                    leftLift.setPower((0));
                    sleep(400);
                    OuttakeFlip.setPosition(0.78);
                    sleep(400);
                    OuttakeSpin.setPosition(.155);
                    sleep(500);
                    rightLift.setPower((.65));
                    leftLift.setPower((-.65));
                    sleep(200);
                    rightLift.setPower((0));
                    leftLift.setPower((0));
                    sleep(600);
                    OuttakeClaw.setPosition(0.275);
                    sleep(500);
                    OuttakeSpin.setPosition(.489);
                    sleep(100);
                    OuttakeFlip.setPosition(0.245);
                    /*drive.followTrajectory(park2);
                    drive.followTrajectory(park);*/




                    ran = false;
                } else if (pos == RedSightPipeline.SkystonePosition.RIGHT) {
                    telemetry.addData("3", pos);
                    TrajectorySequence Left = drive.trajectorySequenceBuilder(new Pose2d())
                            .lineToLinearHeading(new Pose2d(30, -15.5, Math.toRadians(-71)))
                            .back(2)
                            .addTemporalMarker(0, () -> {
                                IntakePos.setPosition(.92);
                            })
                            .addTemporalMarker(1.3, () -> {
                                IntakePos.setPosition(0.89);
                                IntakeUno.setPower((.8));
                                IntakeDos.setPower((-.8));
                                IntakeRoller.setPower((-1));
                            })
                            .addTemporalMarker(6, () -> {
                                IntakePos.setPosition(0.913);
                                IntakeUno.setPower((0));
                                IntakeDos.setPower((0));
                                IntakeRoller.setPower((0));
                            })
                            .waitSeconds(1)
                            .lineToLinearHeading(new Pose2d(18, -35, Math.toRadians(-71)))
                            .addTemporalMarker(3, () -> {
                                sleep(300);
                                OuttakeFlip.setPosition(0.78);
                                sleep(300);
                                OuttakeSpin.setPosition(0);
                            })
                            .addTemporalMarker(5.6, () -> {
                                OuttakeClaw.setPosition(0.275);
                                OuttakeSpin.setPosition(.489);
                            })
                            .forward(5)
                            .waitSeconds(1)
                            .build();

                    Trajectory parkL = drive.trajectoryBuilder(new Pose2d())
                            .strafeRight(25)
                            .build();
                    Trajectory park2L = drive.trajectoryBuilder(new Pose2d())
                            .addTemporalMarker(0, () -> {
                                OuttakeFlip.setPosition(0.245);
                            })
                            .back(3)
                            .build();

                    drive.followTrajectorySequence(Left);


                    drive.followTrajectory(park2L);
                    drive.followTrajectory(parkL);




                    ran = false;
                    //wow its cyancheetah
                }
            }
        }
    }
}