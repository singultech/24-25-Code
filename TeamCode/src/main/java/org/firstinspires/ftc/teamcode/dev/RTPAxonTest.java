package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;
import org.firstinspires.ftc.teamcode.subsystems.RTPAxon;

@TeleOp(name = "RTPAxonTest")
public class RTPAxonTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        CRServo rightServo = hardwareMap.crservo.get("rightFlip");
        AnalogInput rightEncoder = hardwareMap.get(AnalogInput.class, "rightArmEncoder");
        RTPAxon servo = new RTPAxon(rightServo, rightEncoder);

        waitForStart();
        while (!isStopRequested()) {
            gamepads.copyStates();
            servo.update();

            if (gamepads.isPressed("dpad_up")) servo.changeTargetRotation(5);
            if (gamepads.isPressed("dpad_down")) servo.changeTargetRotation(-5);

            telemetry.addLine(servo.log());
            telemetry.update();
        }
    }
}