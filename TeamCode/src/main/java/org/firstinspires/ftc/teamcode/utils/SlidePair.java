package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SlidePair {
    DcMotorEx rightSlide;
    DcMotorEx leftSlide;
    private int currentPosition;
    private boolean isActive;
    private final int maxHeight;
    private double power;

    public SlidePair(DcMotorEx left, DcMotorEx right, int maxH, double startingPower){
        rightSlide = right;
        leftSlide = left;
        maxHeight = maxH;
        power = startingPower;
        init();
    }
    public SlidePair(DcMotorEx left, DcMotorEx right, int maxH){
        rightSlide = right;
        leftSlide = left;
        maxHeight = maxH;
        power = 1.0;
        init();
    }

    private void init(){
        setPower(power);
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPosition(int pos){
        if (pos > maxHeight) {
            leftSlide.setTargetPosition(maxHeight);
            rightSlide.setTargetPosition(maxHeight);
        }
        if (pos < 0){
            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
        }
        leftSlide.setTargetPosition(pos);
        rightSlide.setTargetPosition(pos);
    }

    public void changePosition(int amt){
        if (currentPosition + amt > 0 && currentPosition + amt <= maxHeight){
            currentPosition += amt;
        }
    }

    public void setPower(double p){
        power = p;
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        isActive = (power > 0);
    }

    public double getPower() { return power; }

    public boolean isActive(){
        return isActive;
    }

    public int getPosition(){
        return currentPosition;
    }
}
