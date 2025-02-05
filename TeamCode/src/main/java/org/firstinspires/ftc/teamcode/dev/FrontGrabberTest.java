package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.GamepadPair;
import org.firstinspires.ftc.teamcode.utils.Grabber;
import org.firstinspires.ftc.teamcode.utils.RumbleEffects;

@TeleOp(name = "Front Grabber Test", group = "Dev")
public class FrontGrabberTest extends LinearOpMode {
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Grabber grabber = new Grabber(0.72, 1, hardwareMap.servo.get("frontGrabberServo"), hardwareMap.touchSensor.get("frontGrabberSwitch"));
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        long curTime;
        long lastOpened = 0;

        waitForStart();

        while (opModeIsActive()) {
            curTime = System.currentTimeMillis();
            gamepads.copyStates();

            if (gamepads.isPressed(-1, "cross")) {
                if (grabber.isClosed()) {grabber.open(); lastOpened = curTime;}
                else grabber.close();
            }
            /*
            if (grabber.getSwitchState() && curTime - lastOpened > 3){
                grabber.close();
                gamepads.rumble(-1, 250);

            }*/
            if (grabber.getSwitchState() && curTime - lastOpened > 2000 && !grabber.isClosed()){
                new Thread(() -> {
                    try {
                        grabber.close();
                        gamepads.blipRumble(-1, 1);

                        Thread.sleep(750);

                        if (!grabber.getSwitchState()) {grabber.open(); gamepads.rumble(1, RumbleEffects.alternating);}

                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }).start();
            }

            telemetry.addLine("Press X to open or close the grabber");
            telemetry.addLine(grabber.isClosed() ? "Grabber closed" : "Grabber opened");
            telemetry.addLine(grabber.getSwitchState() ? "Switch pressed" : "Switch not pressed");
            telemetry.update();
        }
    }
}

