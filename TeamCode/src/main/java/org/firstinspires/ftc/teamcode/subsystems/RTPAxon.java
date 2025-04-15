package org.firstinspires.ftc.teamcode.subsystems;


import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;


public class RTPAxon {
    private final AnalogInput servoEncoder;
    private final CRServo servo;
    private boolean rtp;
    private double power;
    private double maxPower;
    private Direction direction;
    private double previousAngle;
    private double totalRotation;
    private double targetRotation;
    private double kP;
    public double STARTPOS;
    public int ntry=0;
    public int cliffs=0;
    public double homeAngle;

    public enum Direction{
        FORWARD,
        REVERSE
    }

    //region constructors
    public RTPAxon(CRServo servo, AnalogInput encoder) {
        rtp = true;
        this.servo = servo;
        servoEncoder = encoder;
        direction = Direction.FORWARD;
        initialize();
    }
    public RTPAxon(CRServo servo, AnalogInput encoder, Direction direction) {
        this(servo, encoder);
        this.direction = direction;
        initialize();
    }
    private void initialize() {

        servo.setPower(0);
        try {
            Thread.sleep(50);
        }
        catch (InterruptedException e){
            e.printStackTrace();
        }

        do {

            STARTPOS = getCurrentAngle();
            if (Math.abs(STARTPOS) > 1) {
                previousAngle = getCurrentAngle();
            } else{
                try {
                    Thread.sleep(50);
                }
                catch (InterruptedException e){
                    e.printStackTrace();
                }

            }
            ntry++;
        }while(Math.abs(previousAngle)<0.2 && (ntry<50));

        totalRotation = 0;
        homeAngle = previousAngle;
        kP = 0.015;
        maxPower = 0.25;

        cliffs = 0 ;   /// This could be wrong if init in not a home position
    }
    //endregion

    //region getters & setters
    public void setDirection(Direction direction){
        this.direction = direction;
    }
    public void setPower(double power) {
        this.power = Math.max(-maxPower, Math.min(maxPower, power));
        servo.setPower(this.power * (direction == Direction.REVERSE ? -1 : 1));
    }
    public double getPower(){
        return power;
    }
    public void setMaxPower(double maxPower){
        this.maxPower = maxPower;
    }
    public double getMaxPower(){
        return maxPower;
    }
    public void setRtp(boolean rtp){
        this.rtp = rtp;
    }
    public boolean getRtp(){
        return rtp;
    }
    public void setK(double k){
        kP = k;
    }
    public double getK() {
        return kP;
    }
    public double getTotalRotation(){
        return totalRotation;
    }
    public double getTargetRotation(){
        return targetRotation;
    }
    public void changeTargetRotation(double change){
        targetRotation += change;
    }
    public void setTargetRotation(double target){
        targetRotation = target;
    }
    public double getCurrentAngle() {
        if (servoEncoder == null) return 0;
        return (servoEncoder.getVoltage() / 3.3) * (direction.equals(Direction.REVERSE) ? -360 : 360);
    }
    public boolean isAtTarget(){
        return Math.abs(targetRotation - totalRotation) < 5;
    }
    public boolean isAtTarget(double tolerance){
        return Math.abs(targetRotation - totalRotation) < tolerance;
    }

    public void forceResetTotalRotation() {
        totalRotation = 0;
        previousAngle = getCurrentAngle();
    }
    //endregion

    public synchronized void update() {
        // This code calculated when the axon has wrapped around its 360 degree encoder limit and adds to the total rotation accordingly.
        double a = getCurrentAngle();
        double angleDifference = a - previousAngle;
        if (angleDifference > 180) {
            angleDifference -= 360;
            cliffs--;
        } else if (angleDifference < -180) {
            angleDifference += 360;
            cliffs++;
        }
        //totalRotation += angleDifference;
        totalRotation = a - homeAngle + cliffs*360;
        previousAngle = a;  //getCurrentAngle();

        // Proportional controller to move the servo to the target rotation.
        if (!rtp) return;
        double error = targetRotation - totalRotation;
        double DEADZONE = 0.5;
        if (Math.abs(error) > DEADZONE) {
            double power = Math.min(maxPower, Math.abs(error * kP)) * Math.signum(error);
            setPower(power);
        } else {
            setPower(0);
        }
    }
    @SuppressLint("DefaultLocale")
    public String log(){
        return String.format("Current Volts: %f\nCurrent Angle: %f\nTotal Rotation: %f\nTarget Rotation: %f\nCurrent Power: %f",
                servoEncoder.getVoltage(),
                getCurrentAngle(), totalRotation, targetRotation, power);
    }


    @TeleOp(name = "Cont. Rotation Axon Test", group = "test")
    public static class CRAxonTest extends LinearOpMode {

        @Override
        public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            CRServo crservo = hardwareMap.crservo.get("rightHorizSlide");
            AnalogInput encoder = hardwareMap.get(AnalogInput.class, "rightHorizSlideEncoder");
            GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
            RTPAxon servo = new RTPAxon(crservo, encoder);
            waitForStart();
            while (!isStopRequested()) {
                gamepads.copyStates();
                servo.update();

                if(gamepads.isPressed(-1, "dpad_up")){
                    servo.changeTargetRotation(15);
                }
                if(gamepads.isPressed(-1, "dpad_down")){
                    servo.changeTargetRotation(-15);
                }
                if(gamepads.isPressed(-1, "cross")){
                    servo.setTargetRotation(0);
                }
                telemetry.addData("Starting angle", servo.STARTPOS);
                telemetry.addLine(servo.log());
                telemetry.addData("NTRY", servo.ntry);
                telemetry.update();
            }
        }
    }
}
