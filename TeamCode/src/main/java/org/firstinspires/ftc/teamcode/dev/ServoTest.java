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
        Servo wristServo = hardwareMap.get(Servo.class, "wristServo");
        Servo armServo = hardwareMap.get(Servo.class, "frontArmServo");
        armServo.setDirection(Servo.Direction.REVERSE);
        double wristTargetPosition = 0.0;
        double armTargetPosition = 0.0;


        waitForStart();
        while (!isStopRequested()) {
            if(gamepads.isPressed("dpad_right")){
                wristTargetPosition +=0.05;
            }
            if(gamepads.isPressed("dpad_left")){
                wristTargetPosition -=0.05;
            }
            if(gamepads.isPressed("circle")){
                armTargetPosition += 0.05;
            }
            if(gamepads.isPressed("square")){
                armTargetPosition -= 0.05;
            }
            wristServo.setPosition(wristTargetPosition);
            armServo.setPosition(armTargetPosition);
            telemetry.addData("Wrist Target Position", wristTargetPosition);
            telemetry.addData("Arm Target Position", armTargetPosition);
            telemetry.update();
        }
    }
}