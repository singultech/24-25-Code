package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Grabber {
    private final Servo grabberServo;
    private final TouchSensor limitSwitch;
    private final double closePosition;
    private final double openPosition;
    private boolean isClosed;


    public Grabber(double close, double open, Servo grabber, TouchSensor limit){
        grabberServo = grabber;
        limitSwitch = limit;
        closePosition = close;
        openPosition = open;
        close();
    }

    public void close(){
        grabberServo.setPosition(closePosition);
        isClosed = true;
    }

    public void open(){
        grabberServo.setPosition(openPosition);
        isClosed = false;
    }

    public boolean isClosed() { return isClosed; }

    public boolean getSwitchState(){
        return limitSwitch.isPressed();
    }

    @TeleOp(name = "Dual Grabber Test", group = "test")
    public static class DualGrabberTest extends LinearOpMode {
        @Override
        public void runOpMode() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            Grabber frontGrabber = new Grabber(0.70, 1, hardwareMap.servo.get("frontGrabberServo"), hardwareMap.touchSensor.get("frontGrabberSwitch"));
            Grabber backGrabber = new Grabber(0.73, 1, hardwareMap.servo.get("backGrabberServo"), hardwareMap.touchSensor.get("backGrabberSwitch"));
            GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
            long curTime;
            long lastFrontOpened = 0;
            long lastBackOpened = 0;

            waitForStart();

            while (opModeIsActive()) {
                curTime = System.currentTimeMillis();
                gamepads.copyStates();

                if (gamepads.isPressed(1, "circle")) {
                    if (frontGrabber.isClosed()) {
                        frontGrabber.open();
                        backGrabber.open();
                        lastFrontOpened = curTime;
                        lastBackOpened = curTime;
                    }
                    else {
                        frontGrabber.close();
                        backGrabber.close();
                    }
                }
                if (frontGrabber.getSwitchState() && curTime - lastFrontOpened > 2000 && !frontGrabber.isClosed()){
                    new Thread(() -> {
                        try {
                            frontGrabber.close();
                            gamepads.blipRumble(-1, 1);

                            Thread.sleep(750);

                            if (!frontGrabber.getSwitchState()) {frontGrabber.open(); gamepads.rumble(1, RumbleEffects.alternating);}

                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                    }).start();
                }
                if (backGrabber.getSwitchState() && curTime - lastBackOpened > 2000 && !backGrabber.isClosed()){
                    new Thread(() -> {
                        try {
                            backGrabber.close();
                            gamepads.blipRumble(1, 1);

                            Thread.sleep(750);

                            if (!backGrabber.getSwitchState()) {backGrabber.open(); gamepads.rumble(1, RumbleEffects.alternating);}

                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                    }).start();
                }

                telemetry.addLine("Press X to open or close the grabber");
                telemetry.addLine(frontGrabber.isClosed() ? "FrontGrabber closed" : "Grabber opened");
                telemetry.addLine(frontGrabber.getSwitchState() ? "FrontSwitch pressed" : "Switch not pressed");
                telemetry.addLine(backGrabber.isClosed() ? "BackGrabber closed" : "Grabber opened");
                telemetry.addLine(backGrabber.getSwitchState() ? "BackSwitch pressed" : "Switch not pressed");
                telemetry.update();
            }
        }
    }

}