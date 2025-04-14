package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class VertSlidePair {
    DcMotorEx rightSlide;
    DcMotorEx leftSlide;
    Hook leftHook;
    Hook rightHook;
    private static final int MAX_HEIGHT = 4000;
    private SlidePosition leftPreset = SlidePosition.ZERO;
    private SlidePosition rightPreset = SlidePosition.ZERO;

    public enum SlidePosition{
        ZERO(0),
        MAX(4000);

        private final int slidePosition;
        SlidePosition(int slidePosition){
            this.slidePosition = slidePosition;
        }
        public int getValue(){
            return slidePosition;
        }
    }
    public enum Side {
        LEFT,
        RIGHT
    }

    public VertSlidePair(double startingPower, HardwareMap hmap){
        rightSlide = hmap.get(DcMotorEx.class, "leftSlide");
        leftSlide = hmap.get(DcMotorEx.class, "rightSlide");
        Servo rightHookServo = hmap.get(Servo.class, "rightHook");
        rightHookServo.setDirection(Servo.Direction.REVERSE);


        leftHook = new Hook(hmap.get(Servo.class, "leftHook"));
        rightHook = new Hook(hmap.get(Servo.class, "rightHook"));
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

    public void setTargetPosition(Side side, SlidePosition preset){
        if (side == Side.LEFT) leftPreset = preset;
        else rightPreset = preset;
        setTargetPosition(side, preset.getValue());
    }
    public void setTargetPosition(SlidePosition preset){
        setTargetPosition(Side.LEFT, preset);
        setTargetPosition(Side.RIGHT, preset);
    }
    public void setTargetPosition(Side side, int position){
        DcMotorEx slide;
        if(side == Side.LEFT) slide = leftSlide;
        else slide = rightSlide;
        int pos = Math.max(0, Math.min(MAX_HEIGHT, position));
        slide.setTargetPosition(pos);
    }
    public void setTargetPosition(int position){
        setTargetPosition(Side.LEFT, position);
        setTargetPosition(Side.RIGHT, position);
    }
    public void changeTargetPosition(Side side, int amt){
        setTargetPosition(side, getTargetPosition(side) + amt);
    }
    public void changeTargetPosition(int amt){
        changeTargetPosition(Side.LEFT, amt);
        changeTargetPosition(Side.RIGHT, amt);
    }

    public void setPower(Side side, double p){
        if(side == Side.LEFT) {
            leftSlide.setPower(Math.max(-1, Math.min(1, p)));
        }
        else {
            rightSlide.setPower(Math.max(-1, Math.min(1, p)));
        }
    }
    public void setPower(double p){
        setPower(Side.LEFT, p);
        setPower(Side.RIGHT, p);
    }

    public double getPower(Side side) {
        if (side == Side.LEFT) return leftSlide.getPower();
        else return rightSlide.getPower();
    }

    public boolean isActive(Side side){
        if (side == Side.LEFT) return (leftSlide.getPower() > 0);
        else return (rightSlide.getPower() > 0);
    }

    public int getTargetPosition(Side side){
        if (side == Side.LEFT) return leftSlide.getTargetPosition();
        else return rightSlide.getTargetPosition();
    }
    public int getPosition(Side side){
        if (side == Side.LEFT) return leftSlide.getCurrentPosition();
        else return rightSlide.getCurrentPosition();
    }
    public SlidePosition getPreset(Side side){
        if (side == Side.LEFT) return leftPreset;
        else return rightPreset;
    }

    private int getIndexOfPreset(SlidePosition preset){
        for(int i = 0; i < SlidePosition.values().length; i++){
            if(SlidePosition.values()[i] == preset) return i;
        }
        return -1;
    }
    public void incrementSlidePreset(Side side, int amount){
        int currentPresetIndex = getIndexOfPreset(getPreset(side));
        if (currentPresetIndex == -1) return;
        if (currentPresetIndex + amount < 0 || currentPresetIndex + amount >= SlidePosition.values().length) return;
        SlidePosition newPreset = SlidePosition.values()[currentPresetIndex + amount];
        setTargetPosition(side, newPreset);
    }
    public void incrementSlidePreset(int amount){
        incrementSlidePreset(Side.LEFT, amount);
        incrementSlidePreset(Side.RIGHT, amount);
    }

    public void resetPosition(){
        setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void performTimedMove(Side side, int changeOfPosition, Integer finalPosition, long waitTimeMillis) {
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

    public void setHook(Side side, Hook.HookPosition position){
        if (side == Side.LEFT) leftHook.setPosition(position);
        else rightHook.setPosition(position);
    }
    public void setHook(Hook.HookPosition position){
        setHook(Side.LEFT, position);
        setHook(Side.RIGHT, position);
    }
    public Hook.HookPosition getHook(Side side){
        if (side == Side.LEFT) return leftHook.getPosition();
        else return rightHook.getPosition();
    }

    @NonNull
    @Override
    public String toString(){
        String log = "Vertical Slide Pair:\nLeft Target: %d, Left Current: %d\nRight Target: %d, Right Current: %d";
        return String.format(log,
                leftSlide.getTargetPosition(),
                leftSlide.getCurrentPosition(),
                rightSlide.getTargetPosition(),
                rightSlide.getCurrentPosition()
        );
    }

    @TeleOp(name = "Vert Slides Test", group = "Dev")
    public static class VertSlidesTest extends LinearOpMode {
        static final double SLIDE_POWER = 1.0;
        @Override
        public void runOpMode() {

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            VertSlidePair slides = new VertSlidePair(hardwareMap);
            GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);

            waitForStart();

            while (opModeIsActive()) {

                if (gamepads.isPressed(-1, "square")) {
                    slides.resetPosition();
                }
                if (gamepads.isPressed("right_bumper")){
                    if (slides.isActive(Side.RIGHT)){
                        slides.setPower(Side.RIGHT, 0);
                    } else{
                        slides.setPower(Side.RIGHT, SLIDE_POWER);
                    }
                }
                if (gamepads.isPressed("left_bumper")){
                    if (slides.isActive(Side.LEFT)){
                        slides.setPower(Side.LEFT, 0);
                    } else{
                        slides.setPower(Side.LEFT, SLIDE_POWER);
                    }
                }
                if (gamepads.isPressed("triangle")){
                    slides.setHook(Hook.HookPosition.DOWN);
                }
                if (gamepads.isHeld("dpad_up")){
                    slides.changeTargetPosition(Side.LEFT,30);
                }
                if (gamepads.isHeld("dpad_down")){
                    slides.changeTargetPosition(Side.LEFT,-30);
                }
                if (gamepads.isHeld("triangle")){
                    slides.changeTargetPosition(Side.RIGHT,30);
                }
                if (gamepads.isHeld("cross")){
                    slides.changeTargetPosition(Side.RIGHT,-30);
                }

                telemetry.addLine("Use the D-pad and up and down face buttons to control the slides.");
                telemetry.addLine("Press ▣ to reset the slides position to 0.");
                telemetry.addLine("Press Either Bumper to toggle the holding motors for a side");
                telemetry.addLine(slides.toString());
                telemetry.update();
            }
        }
    }
}