package org.firstinspires.ftc.teamcode;
/**
 * @author CyanCheeah
 * This is motor constants for the elevator and bucket and servo.
 */
public class MotorConstantValues {

    private double triggerPowerAdjust = 1;

    private double intakeUp = 0.85;

    private double intakeDown = 0.99;

    private double outClose = 0.03;

    private double outOpen = 0.275;

    private double flipOut = 0.78;

    private double flipIn = 0.245;

    private double spinOne = 0.3125;

    private double spinTwo = 0.655;
    public double getIntakeUp(){
        return intakeUp;
    }
    public double getIntakeDown(){
        return intakeDown;
    }
    public double getOutClose(){
        return outClose;
    }
    public double getOutOpen(){
        return outOpen;
    }
    public double getFlipOut(){
        return flipOut;
    }
    public double getFlipIn(){
        return flipIn;
    }
    public double getSpinOne(){
        return spinOne;
    }
    public double getSpinTwo(){
        return spinTwo;
    }


}
