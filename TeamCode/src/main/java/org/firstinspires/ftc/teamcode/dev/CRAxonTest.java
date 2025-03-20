package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.subsystems.RTPAxon;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "Cont. Rotation Axon Test", group = "Testing")
public class CRAxonTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CRServo crservo = hardwareMap.crservo.get("leftDiffy");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "leftDiffyEncoder");
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

            telemetry.addLine(servo.log());
            telemetry.update();
        }
    }
}