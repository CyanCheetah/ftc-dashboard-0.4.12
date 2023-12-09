package org.firstinspires.ftc.teamcode;
/**
 * @author CyanCheeah
 * This is motor constants for the elevator and bucket and servo.
 */
public class MotorConstantValues {
    private double SwingOutPosition = 0;

    private double SwingScorePosition = 0.275;

    private double SwingInPosition = 0.31;
    private double BucketOutPosition = .16;

    private double BucketInPosition = BucketOutPosition + .155;
    private double clawClose = .95;
    private double clawSemiOpen = .9;
    private double clawFullOpen = .75;
    public double getClawClose() {
        return clawClose;
    }
    public double getClawSemiOpen() {
        return clawSemiOpen;
    }
    public double getClawFullOpen() {
        return clawFullOpen;
    }
    public double getSwingOutPosition() {
        return SwingOutPosition;
    }
    public double getSwingScorePosition() {
        return SwingScorePosition;
    }
    public double getSwingInPosition() {
        return SwingInPosition;
    }
    public double getBucketOutPosition() {
        return BucketOutPosition;
    }
    public double getBucketInPosition() {
        return BucketInPosition;
    }

}
