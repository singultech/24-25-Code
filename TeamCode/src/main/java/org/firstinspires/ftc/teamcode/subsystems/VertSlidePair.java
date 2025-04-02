package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VertSlidePair {
    DcMotorEx rightSlide;
    DcMotorEx leftSlide;
    private static final int MAX_HEIGHT = 3000;
    private SlidePosition leftPreset = SlidePosition.ZERO;
    private SlidePosition rightPreset = SlidePosition.ZERO;

    public enum SlidePosition{
        ZERO(0),
        GRAB_FROM_WALL(100),
        HANG_SPECIMEN(1000),
        HANG_ROBOT(2000);

        private final int slidePosition;
        SlidePosition(int slidePosition){
            this.slidePosition = slidePosition;
        }
        public int getValue(){
            return slidePosition;
        }
    }
    public enum SlideSide{
        LEFT,
        RIGHT
    }

    public VertSlidePair(double startingPower, HardwareMap hmap){
        rightSlide = hmap.get(DcMotorEx.class, "rightSlide");
        leftSlide = hmap.get(DcMotorEx.class, "leftSlide");
        setPower(startingPower);
        init();
    }
    public VertSlidePair(HardwareMap hmap){
        this(1, hmap);
    }

    private void init(){
        setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setTargetPosition(SlideSide side, SlidePosition preset){
        if (side == SlideSide.LEFT) leftPreset = preset;
        else rightPreset = preset;
        setTargetPosition(side, preset.getValue());
    }
    public void setTargetPosition(SlidePosition preset){
        setTargetPosition(SlideSide.LEFT, preset);
        setTargetPosition(SlideSide.RIGHT, preset);
    }
    public void setTargetPosition(SlideSide side, int position){
        DcMotorEx slide;
        if(side == SlideSide.LEFT) slide = leftSlide;
        else slide = rightSlide;
        int pos = Math.max(0, Math.min(MAX_HEIGHT, position));
        slide.setTargetPosition(pos);
    }
    public void setTargetPosition(int position){
        setTargetPosition(SlideSide.LEFT, position);
        setTargetPosition(SlideSide.RIGHT, position);
    }
    public void changeTargetPosition(SlideSide side, int amt){
        setTargetPosition(side, getTargetPosition(side) + amt);
    }
    public void changeTargetPosition(int amt){
        changeTargetPosition(SlideSide.LEFT, amt);
        changeTargetPosition(SlideSide.RIGHT, amt);
    }

    public void setPower(SlideSide side, double p){
        if(side == SlideSide.LEFT) {
            leftSlide.setPower(Math.max(-1, Math.min(1, p)));
        }
        else {
            rightSlide.setPower(Math.max(-1, Math.min(1, p)));
        }
    }
    public void setPower(double p){
        setPower(SlideSide.LEFT, p);
        setPower(SlideSide.RIGHT, p);
    }

    public double getPower(SlideSide side) {
        if (side == SlideSide.LEFT) return leftSlide.getPower();
        else return rightSlide.getPower();
    }

    public boolean isActive(SlideSide side){
        if (side == SlideSide.LEFT) return (leftSlide.getPower() > 0);
        else return (rightSlide.getPower() > 0);
    }

    public int getTargetPosition(SlideSide side){
        if (side == SlideSide.LEFT) return leftSlide.getTargetPosition();
        else return rightSlide.getTargetPosition();
    }
    public int getPosition(SlideSide side){
        if (side == SlideSide.LEFT) return leftSlide.getCurrentPosition();
        else return rightSlide.getCurrentPosition();
    }
    public SlidePosition getPreset(SlideSide side){
        if (side == SlideSide.LEFT) return leftPreset;
        else return rightPreset;
    }

    public void resetPosition(){
        setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void performTimedMove(SlideSide side, int changeOfPosition, Integer finalPosition, long waitTimeMillis) {
        int startPosition = getPosition(side);
        int positionToGo = startPosition + changeOfPosition;
        new Thread(() -> {
            try {
                setTargetPosition(side, positionToGo);

                while (Math.abs(getPosition(side) - positionToGo) > 20) {
                    Thread.sleep(20);
                }

                Thread.sleep(waitTimeMillis);

                if (finalPosition != null) {
                    setTargetPosition(side, finalPosition);

                    while (Math.abs(getPosition(side) - finalPosition) > 20) {
                        Thread.sleep(20);
                    }
                }

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
    }

    @NonNull
    @SuppressLint("DefaultLocale")
    public String toString(){
        String log = "Vertical Slide Pair:\nLeft Target: %f, Left Current: %f\nRight Target: %f, Right Current: %f";
        return String.format(log ,leftSlide.getTargetPosition(), leftSlide.getCurrentPosition(), rightSlide.getTargetPosition(), rightSlide.getCurrentPosition());
    }
}