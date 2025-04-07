package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "ServoTest")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Servo servo = hardwareMap.get(Servo.class, "wristServo");
        //servo.setDirection(Servo.Direction.REVERSE);
        double targetPosition = 0.0;

        waitForStart();
        while (!isStopRequested()) {
            if(gamepads.isPressed("dpad_right")){
                targetPosition+=0.05;
            }
            if(gamepads.isPressed("dpad_left")){
                targetPosition-=0.05;
            }
            if(gamepads.isPressed("dpad_up")) targetPosition=0.95;
            if(gamepads.isPressed("dpad_down")) targetPosition=0.23;
            servo.setPosition(targetPosition);
            telemetry.addData("Target Position", targetPosition);
            telemetry.update();
        }
    }
}