package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Arm;
import org.firstinspires.ftc.teamcode.utils.GamepadPair;

@TeleOp(name = "Arm Test", group = "Dev")
public class ArmTest extends LinearOpMode {
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Arm arm = new Arm(1, 0, hardwareMap);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();

            if (gamepads.isHeld(-1, "cross")) {
                arm.setPower(1);
            } else if (gamepads.isHeld(-1, "circle")) arm.setPower(-1);
            else arm.setPower(0);

            telemetry.addData("Current Angle", arm.getAngle());
            telemetry.addData("TotalRotation", arm.getPosition());
            telemetry.update();
        }
    }
}

