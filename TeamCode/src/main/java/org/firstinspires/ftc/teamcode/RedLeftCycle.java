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
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
        RedSightPipeline.SkystonePosition pos;
        while (!isStarted() && !isStopRequested()) {
            pos = pipeline.getAnalysis();
            telemetry.addData("Color", pos);
            telemetry.update();
        }
        pos = pipeline.getAnalysis();
        waitForStart();
        if (opModeIsActive()) {
            telemetry.addData("Color", pos);
            telemetry.update();
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            if (ran) {
                if (pos == RedSightPipeline.SkystonePosition.LEFT) {
                    telemetry.addData("1", pos);
                    TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                            .forward(23)//change to forward turn
                            .addTemporalMarker(0, () -> {
                                IntakePos.setPosition(.92);
                            })
                            .lineToLinearHeading(new Pose2d(17, 24, Math.toRadians(-75)))
                            .addTemporalMarker(2, () -> {
                                IntakeUno.setPower((-.8));
                                IntakeDos.setPower((.8));
                                IntakeRoller.setPower((1));
                            })
                            .addTemporalMarker(3.5, () -> {
                                IntakeUno.setPower((0));
                                IntakeDos.setPower((0));
                                IntakeRoller.setPower((0));
                            })
                            .build();
                    TrajectorySequence trajectoryMiddle2 = drive.trajectorySequenceBuilder(new Pose2d())
                            // .splineTo(new Vector2d(-5, 24), Math.toRadians(0))
                            .splineTo(new Vector2d(10, 20), Math.toRadians(3))
                            .addTemporalMarker(0, () -> {
                                IntakeUno.setPower((-.8));
                                IntakeDos.setPower((.8));
                                IntakeRoller.setPower((1));
                            })
                            .splineTo(new Vector2d(70, 20), Math.toRadians(0))
                            .addTemporalMarker(1, () -> {
                                IntakeUno.setPower((0));
                                IntakeDos.setPower((0));
                                IntakeRoller.setPower((0));
                            })
                            .splineTo(new Vector2d(95, 8), Math.toRadians(0))
                            //.lineToLinearHeading(new Pose2d(15, -20, Math.toRadians(0)))
                            .build();
                    /*
                    Trajectory trajectoryMiddle5 = drive.trajectoryBuilder(new Pose2d())
                            .lineToLinearHeading(new Pose2d(-15, 20, Math.toRadians(0)))
                            .build();
                    Trajectory trajectoryMiddle6 = drive.trajectoryBuilder(new Pose2d())
                            .back(65)
                            .build();
                            */
                    //drive.followTrajectory(trajectoryMiddle0);
                    drive.followTrajectorySequence(trajSeq);
                    //drive.followTrajectory(trajectoryMiddle1);
                    //drive.followTrajectory(trajectoryMiddle9);
                    drive.followTrajectorySequence(trajectoryMiddle2);
                    //drive.followTrajectory(trajectoryMiddle3);
                    //drive.followTrajectory(trajectoryMiddle4);
                    sleep(1000);
                    //drive.followTrajectory(trajectoryMiddle5);
                    //drive.followTrajectory(trajectoryMiddle6);
                    ran = false;

                } else if (pos == RedSightPipeline.SkystonePosition.CENTER) {
                    telemetry.addData("2", pos);
                    TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                            .forward(21)
                            .addTemporalMarker(0, () -> {
                                IntakePos.setPosition(.925);
                            })
                            .lineToLinearHeading(new Pose2d(17, 24, Math.toRadians(-75)))
                            .addTemporalMarker(2, () -> {
                                IntakeUno.setPower((-.8));
                                IntakeDos.setPower((.8));
                                IntakeRoller.setPower((1));
                            })
                            .addTemporalMarker(3.5, () -> {
                                IntakeUno.setPower((0));
                                IntakeDos.setPower((0));
                                IntakeRoller.setPower((0));
                            })
                            .build();
                    TrajectorySequence trajectoryMiddle2 = drive.trajectorySequenceBuilder(new Pose2d())
                            .splineTo(new Vector2d(10, 20), Math.toRadians(3))
                            .addTemporalMarker(0, () -> {
                                IntakeUno.setPower((-.8));
                                IntakeDos.setPower((.8));
                                IntakeRoller.setPower((1));
                            })
                            .splineTo(new Vector2d(70, 20), Math.toRadians(0))
                            .addTemporalMarker(1, () -> {
                                IntakeUno.setPower((0));
                                IntakeDos.setPower((0));
                                IntakeRoller.setPower((0));
                            })
                            .splineTo(new Vector2d(95, 8), Math.toRadians(0))//ends when its in middle position
                            .build();
                    drive.followTrajectorySequence(trajSeq);
                    // drive.followTrajectorySequence(trajectoryMiddle2);
                    ran = false;
                } else if (pos == RedSightPipeline.SkystonePosition.RIGHT) {
                    telemetry.addData("3", pos);
                    TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                            .forward(23)
                            .addTemporalMarker(0, () -> {
                                IntakePos.setPosition(.92);
                            })
                            .lineToLinearHeading(new Pose2d(17, 24, Math.toRadians(-75)))
                            .addTemporalMarker(2, () -> {
                                IntakeUno.setPower((-.8));
                                IntakeDos.setPower((.8));
                                IntakeRoller.setPower((1));
                            })
                            .addTemporalMarker(3.5, () -> {
                                IntakeUno.setPower((0));
                                IntakeDos.setPower((0));
                                IntakeRoller.setPower((0));
                            })
                            .build();
                    TrajectorySequence trajectoryMiddle2 = drive.trajectorySequenceBuilder(new Pose2d())
                            // .splineTo(new Vector2d(-5, 24), Math.toRadians(0))
                            .splineTo(new Vector2d(10, 20), Math.toRadians(3))
                            .addTemporalMarker(0, () -> {
                                IntakeUno.setPower((-.8));
                                IntakeDos.setPower((.8));
                                IntakeRoller.setPower((1));
                            })
                            .splineTo(new Vector2d(70, 20), Math.toRadians(0))
                            .addTemporalMarker(1, () -> {
                                IntakeUno.setPower((0));
                                IntakeDos.setPower((0));
                                IntakeRoller.setPower((0));
                            })
                            .splineTo(new Vector2d(95, 8), Math.toRadians(0))
                            .build();
                    drive.followTrajectorySequence(trajSeq);
                    drive.followTrajectorySequence(trajectoryMiddle2);
                    //drive.followTrajectory(trajectoryMiddle3);
                    //drive.followTrajectory(trajectoryMiddle4);
                    sleep(1000);
                    //drive.followTrajectory(trajectoryMiddle5);
                    //drive.followTrajectory(trajectoryMiddle6);
                    ran = false;
                    //wow its cyancheetah
                }
            }
        }
    }
}