package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.utils.DiffyGrabber;
import org.firstinspires.ftc.teamcode.utils.FrontGrabber;
import org.firstinspires.ftc.teamcode.utils.GamepadPair;

@TeleOp(name = "Diffy Test", group = "Dev")
public class DiffyTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        DiffyGrabber diffy = new DiffyGrabber(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepads.isPressed(-1, "y")) {
                diffy.changeRightPosition(0.1);
            }
            if (gamepads.isPressed(-1, "dpad_up")){
                diffy.changeLeftPosition(0.01);
            }

            if (gamepads.isPressed(-1, "a")){
                diffy.changeRightPosition(-0.1);
            }

            if (gamepads.isPressed(-1, "dpad_down")){
                diffy.changeLeftPosition(-0.1);
            }

            telemetry.addData("right angle", diffy.getRightPosition());
            telemetry.addData("left angle", diffy.getLeftPosition());
            telemetry.addData("right target", diffy.getRightTarget());
            telemetry.addData("left target", diffy.getLeftTarget());
            telemetry.update();
        }
    }
}

