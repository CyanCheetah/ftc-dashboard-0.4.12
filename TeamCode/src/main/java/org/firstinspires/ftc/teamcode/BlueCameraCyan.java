
/*
 * Copyright (c) 2020 OpenFTC Team
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

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.acmerobotics.roadrunner.Pose2d;
        import com.acmerobotics.roadrunner.Vector2d;

// TODO: remove Actions from the core module?
        import com.acmerobotics.roadrunner.ftc.Actions;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import org.firstinspires.ftc.teamcode.MecanumDrive;
        import org.firstinspires.ftc.teamcode.TankDrive;
        import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.opencv.core.Core;
        import org.opencv.core.Mat;
        import org.opencv.core.Point;
        import org.opencv.core.Rect;
        import org.opencv.core.Scalar;
        import org.opencv.imgproc.Imgproc;
        import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvCameraRotation;
        import org.openftc.easyopencv.OpenCvInternalCamera;
        import org.openftc.easyopencv.OpenCvWebcam;
        import org.openftc.easyopencv.OpenCvPipeline;

        import com.acmerobotics.roadrunner.Pose2d;
        import com.acmerobotics.roadrunner.Vector2d;

// TODO: remove Actions from the core module?
        import com.acmerobotics.roadrunner.ftc.Actions;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import org.firstinspires.ftc.teamcode.MecanumDrive;
        import org.firstinspires.ftc.teamcode.TankDrive;
        import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */
@Autonomous
public class BlueCamera1 extends LinearOpMode {
    //ooh
    OpenCvWebcam webcam;


    //dang so this works huh, thats wild
    //yoo

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(200, 550);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(925, 450);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1550, 550);
    static final int REGION_WIDTH = 150;
    static final int REGION_HEIGHT = 150;
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
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region1_Cb, region2_Cb, region3_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1, avg2, avg3;

    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }


    //higgfhhj
    @Override
    public void runOpMode() {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cameraMonitorViewId"), cameraMonitorViewId);


        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        boolean stuffy = true;
        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        waitForStart();
        while (opModeIsActive()) {
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);
            if (stuffy) {
                if (max == avg1) {
                    telemetry.addData("Left", avg1);
                    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .lineToX(5)
                                    .build());
                    stuffy = false;
                } else if (max == avg2) {
                    telemetry.addData("Middle", avg2);
                    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .lineToY(30)
                                    .build());
                    stuffy = false;
                } else if (max == avg3) {
                    telemetry.addData("Right", avg3);
                    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .lineToY(-30)
                                    .build());
                    stuffy = false;
                } else {
                    telemetry.addData("Not found", avg1);
                }
                telemetry.addData("LEftVal", avg1);
                telemetry.addData("mid", avg2);
                telemetry.addData("righ", avg3);
                telemetry.update();

                // Don't burn CPU cycles busy-looping in this sample
                sleep(50);

                telemetry.update();
            }
            telemetry.addData("Pipelinr npy founf", 5);
            telemetry.update();
        }
    }

}