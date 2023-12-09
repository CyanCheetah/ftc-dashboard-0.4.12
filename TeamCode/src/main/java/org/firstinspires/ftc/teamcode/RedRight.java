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
            double clawFullOpen = .775;
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            //multiply this number by the inches needed to travel: 0.68571429
            if (ran) {
                if (max == first) {
                    telemetry.addData("1", first);
                    Trajectory trajectoryFirst0 = drive.trajectoryBuilder(new Pose2d())
                            .forward(20)
                            .build();
                    TrajectorySequence ts = drive.trajectorySequenceBuilder(new Pose2d())
                            .turn(Math.toRadians(-102)) // Turns 45 degrees counter-clockwise
                            .build();
                    Trajectory trajectoryFirst2 = drive.trajectoryBuilder(new Pose2d())
                            .back(2)
                            .addTemporalMarker(3, () -> {
                                Turn.setPosition(.75);
                            })
                            .build();
                    Trajectory trajectoryFirst3 = drive.trajectoryBuilder(new Pose2d())
                            .lineToConstantHeading(new Vector2d(31,16))
                            .build();
                    Trajectory park = drive.trajectoryBuilder(new Pose2d())
                            .strafeRight(35)
                            .build();
                    Trajectory park0 = drive.trajectoryBuilder(new Pose2d())
                            .back(3)
                            .build();
                    Trajectory back1 = drive.trajectoryBuilder(new Pose2d())
                            .back(3)
                            .build();
                    drive.followTrajectory(trajectoryFirst0);
                    drive.followTrajectorySequence(ts);
                    drive.followTrajectory(trajectoryFirst2);
                    sleep(1000);
                    drive.followTrajectory(trajectoryFirst3);
                    drive.followTrajectory(back1);
                    Bucket.setPosition(.26);
                    leftLift.setPower(.3);
                    rightLift.setPower(-.3);
                    sleep(2300);
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    Swing.setPosition(0);
                    sleep(2000);
                    Bucket.setPosition(.11);
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
                    telemetry.addData("hi", "SecondOne");
                    telemetry.addData("2", second);
                    Trajectory trajectoryMiddle0 = drive.trajectoryBuilder(new Pose2d())
                            .forward(31.5)
                            .addTemporalMarker(4, () -> {
                                Turn.setPosition(.75);
                            })
                            .build();
                    Trajectory trajectoryMiddle1 = drive.trajectoryBuilder(new Pose2d())
                            .forward(5)
                            .build();
                    Trajectory trajectoryMiddle2 = drive.trajectoryBuilder(new Pose2d())
                            .lineToLinearHeading(new Pose2d(0, -25, Math.toRadians(-77)))
                            .build();
                    Trajectory trajectoryMiddle3 = drive.trajectoryBuilder(new Pose2d())
                            .strafeRight(30)
                            .build();
                    Trajectory trajectoryMiddle4 = drive.trajectoryBuilder(new Pose2d())
                            .forward(11.5)
                            .build();
                    Trajectory trajectoryMiddle5 = drive.trajectoryBuilder(new Pose2d())
                            .back(7)
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
                    Bucket.setPosition(.3);
                    leftLift.setPower(.3);
                    rightLift.setPower(-.3);
                    sleep(2300);
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    Swing.setPosition(0);
                    sleep(2000);
                    Bucket.setPosition(.11);
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
                    Trajectory trajectoryRight0 = drive.trajectoryBuilder(new Pose2d())
                            .lineToConstantHeading(new Vector2d(23, -22))
                            .build();
                    Trajectory rightStr = drive.trajectoryBuilder(new Pose2d())
                            .strafeRight(6)
                            .addTemporalMarker(1.5, () -> {
                                Turn.setPosition(-.75);
                            })
                            .build();
                    Trajectory trajectoryRight1 = drive.trajectoryBuilder(new Pose2d())
                            .lineToLinearHeading(new Pose2d(-30, -34.5, Math.toRadians(-77)))
                            .build();

                    Trajectory trajectoryRight2 = drive.trajectoryBuilder(new Pose2d())
                            .forward(10)
                            .build();
                    Trajectory rightpark = drive.trajectoryBuilder(new Pose2d())
                            .back(7)
                            .build();
                    Trajectory rightpark2 = drive.trajectoryBuilder(new Pose2d())
                            .strafeRight(29)
                            .build();
                    Trajectory back3 = drive.trajectoryBuilder(new Pose2d())
                            .back(3)
                            .build();
                    drive.followTrajectory(trajectoryRight0);
                    drive.followTrajectory(rightStr);
                    drive.followTrajectory(trajectoryRight2);
                    drive.followTrajectory(trajectoryRight1);
                    drive.followTrajectory(back3);
                    sleep(1000);
                    Bucket.setPosition(.3);
                    leftLift.setPower(.3);
                    rightLift.setPower(-.3);
                    sleep(2300);
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    Swing.setPosition(0);
                    sleep(2000);
                    Bucket.setPosition(.11);
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

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    //wow its cyancheetah
    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum SkystonePosition
        {
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(180, 470);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(925, 450);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1720, 470);
        static final int REGION_WIDTH = 250;
        static final int REGION_HEIGHT = 150;


        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                1920,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;
        boolean pos1 = false;
        boolean pos2 = false;
        boolean pos3 = false;

        // Volatile since accessed by OpMode thread w/o synchronization

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }
        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(max == avg1) // Was it from region 1?
            {

                pos1 = true;
                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg2) // Was it from region 2?
            {
                pos2 = true;
                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg3) // Was it from region 3?
            {
                pos3 = true;
                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }


            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }


        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */


        public int isPos1() {
            return avg1;
        }
        public int isPos2() {
            return avg2;
        }
        public int isPos3() {
            return avg3;
        }
    }



}//