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
 * This is the BlueRight autonomous that scores only on purple.
 */
package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.qualcomm.robotcore.hardware.CRServo;
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
public class BlueLeft extends LinearOpMode
{
    private static CRServo IntakeUno = null;
    private static CRServo IntakeDos = null;
    private static CRServo IntakeRoller = null;
    private static Servo IntakePos = null;
    private static Servo OuttakeClaw = null;
    private static Servo OuttakeFlip = null;
    private static Servo OuttakeSpin = null;

    //  private static Servo servoTwo = null;
    //  private static Servo Turn = null;
    OpenCvWebcam webcam;
    BlueSightPipeline pipeline = new BlueSightPipeline(telemetry);
    private static DcMotor rightLift = null;
    private static DcMotor leftLift = null;
    //  private static Servo Bucket = null;
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
        webcam.setPipeline(pipeline);

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

        //leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*
         * Wait for the user to press start on the Driver Station
         */
        IntakeUno = hardwareMap.get(CRServo.class, "IntakeUno");
        IntakeDos = hardwareMap.get(CRServo.class, "IntakeDos");
        IntakeRoller = hardwareMap.get(CRServo.class, "IntakeRoller");
        IntakePos = hardwareMap.get(Servo.class, "IntakePos");
        leftLift = hardwareMap.get(DcMotor.class,"leftLift");
        rightLift = hardwareMap.get(DcMotor.class,"rightLift");
        OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
        OuttakeFlip = hardwareMap.get(Servo.class, "OuttakeFlip");
        OuttakeSpin = hardwareMap.get(Servo.class, "OuttakeSpin");

        BlueSightPipeline.SkystonePosition pos;
        while (!isStarted() && !isStopRequested()) {
            pos = pipeline.getAnalysis();
            telemetry.addData("Color", pos);
            telemetry.update();
        }
        pos = pipeline.getAnalysis();
        OuttakeClaw.setPosition(0);
        waitForStart();
        if (opModeIsActive())
        {
            pos = pipeline.getAnalysis();
            boolean ran = true;
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            //multiply this number by the inches needed to travel: 0.68571429
            if (ran) {
                if (pos == BlueSightPipeline.SkystonePosition.LEFT) {
                    telemetry.addData("1", pos);
                    TrajectorySequence Left = drive.trajectorySequenceBuilder(new Pose2d())
                            .lineToLinearHeading(new Pose2d(28, 15.5, Math.toRadians(71)))
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
                            .lineToLinearHeading(new Pose2d(12, 43, Math.toRadians(71)))
                            .addTemporalMarker(5, () -> {
                                OuttakeClaw.setPosition(0);
                                rightLift.setPower((-.65));
                                leftLift.setPower((.65));
                                sleep(40);
                                rightLift.setPower((0));
                                leftLift.setPower((0));
                            })
                            .forward(5)
                            .build();

                    Trajectory parkL = drive.trajectoryBuilder(new Pose2d())
                            .strafeLeft(20)
                            .build();
                    Trajectory park2L = drive.trajectoryBuilder(new Pose2d())
                            .addTemporalMarker(0, () -> {
                                OuttakeFlip.setPosition(0.245);
                            })
                            .back(3)
                            .build();

                    drive.followTrajectorySequence(Left);
                    //  drive.followTrajectorySequence(trajectoryMiddle2);

                    OuttakeFlip.setPosition(0.78);
                    sleep(1000);
                    OuttakeClaw.setPosition(0.275);
                    sleep(500);

                    drive.followTrajectory(park2L);
                    drive.followTrajectory(parkL);




                    ran = false;
                } else if (pos == BlueSightPipeline.SkystonePosition.CENTER) {
                    telemetry.addData("2", pos);
                    TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                            .lineToLinearHeading(new Pose2d(34, 0, Math.toRadians(140)))

                            .addTemporalMarker(0, () -> {
                                IntakePos.setPosition(.913);
                            })
                            .addTemporalMarker(2.5, () -> {
                                IntakePos.setPosition(0.925);
                            })
                            .addTemporalMarker(2.35, () -> {
                                IntakePos.setPosition(0.93);
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
                            .lineToLinearHeading(new Pose2d(64, 43, Math.toRadians(71)))
                            .addTemporalMarker(5, () -> {
                                OuttakeClaw.setPosition(0);
                                rightLift.setPower((-.65));
                                leftLift.setPower((.65));
                                sleep(40);
                                rightLift.setPower((0));
                                leftLift.setPower((0));
                            })
                            .forward(5)


                            .build();

                    Trajectory park = drive.trajectoryBuilder(new Pose2d())
                            .strafeLeft(29)
                            .build();
                    Trajectory park2 = drive.trajectoryBuilder(new Pose2d())
                            .addTemporalMarker(0, () -> {
                                OuttakeFlip.setPosition(0.245);
                            })
                            .back(3)
                            .build();

                    drive.followTrajectorySequence(trajSeq);
                    OuttakeFlip.setPosition(0.78);//try adding to temporal marker
                    sleep(1000);
                    OuttakeClaw.setPosition(0.275);
                    sleep(400);
                    drive.followTrajectory(park2);
                    drive.followTrajectory(park);




                    ran = false;


                } else if (pos == BlueSightPipeline.SkystonePosition.RIGHT) {
                    telemetry.addData("1", pos);
                    TrajectorySequence Left = drive.trajectorySequenceBuilder(new Pose2d())
                            .forward(19)
                            .turn(Math.toRadians(71))
                            .back(8)
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
                            .lineToLinearHeading(new Pose2d(16, 43, Math.toRadians(71)))
                            .addTemporalMarker(5, () -> {
                                OuttakeClaw.setPosition(0);
                                rightLift.setPower((-.65));
                                leftLift.setPower((.65));
                                sleep(40);
                                rightLift.setPower((0));
                                leftLift.setPower((0));
                            })
                            .forward(5)
                            .build();

                    Trajectory parkL = drive.trajectoryBuilder(new Pose2d())
                            .strafeLeft(20)
                            .build();
                    Trajectory park2L = drive.trajectoryBuilder(new Pose2d())
                            .addTemporalMarker(0, () -> {
                                OuttakeFlip.setPosition(0.245);
                            })
                            .back(3)
                            .build();

                    drive.followTrajectorySequence(Left);

                    OuttakeFlip.setPosition(0.78);
                    sleep(1000);
                    OuttakeClaw.setPosition(0.275);
                    sleep(500);

                    drive.followTrajectory(park2L);
                    drive.followTrajectory(parkL);




                    ran = false;
                }
            }
        }
    }
}

/*
Trajectory trajectoryMiddle3 = drive.trajectoryBuilder(new Pose2d())
                            .back(10)
                            .build();
                    Trajectory trajectoryMiddle4 = drive.trajectoryBuilder(new Pose2d())
                            .strafeLeft(20)
                            .build();
                    Trajectory trajectoryMiddle5 = drive.trajectoryBuilder(new Pose2d())
                            .forward(4)
                            .addTemporalMarker(3, () -> {
                                OuttakeClaw.setPosition(0);
                                sleep(100);
                                rightLift.setPower((-.65));
                                leftLift.setPower((.65));
                                sleep(300);
                                rightLift.setPower((0));
                                leftLift.setPower((0));
                                sleep(100);

                            })
                            .build();
 */