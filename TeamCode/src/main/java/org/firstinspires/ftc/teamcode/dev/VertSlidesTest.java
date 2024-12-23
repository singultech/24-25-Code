package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.GamepadPair;
import org.firstinspires.ftc.teamcode.utils.VertSlidePair;

@TeleOp(name = "Vert Slides Test", group = "Dev")
public class VertSlidesTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        double slidePower = 1.0;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        VertSlidePair slides = new VertSlidePair(4100, slidePower, hardwareMap);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepads.isPressed(-1, "square")) {
                slides.resetPosition();
            }
            if (gamepads.isPressed(-1, "triangle")){
                if (slides.isActive()){
                    slides.setPower(0);
                } else{
                    slides.setPower(slidePower);
                }
            }
            if (gamepad1.dpad_up){
                slides.changeTargetPosition(50);
            }
            if (gamepad1.dpad_down){
                slides.changeTargetPosition(-50);
            }
            //if (gamepads.isPressed(1, "cross")){
            //slides.performCycleMove(2000, 3000);
            //}

            telemetry.addLine("Use the D-pad to control the slides.");
            telemetry.addLine("Press ▣ to reset the slides position to 0.");
            telemetry.addLine("Press ▲ to toggle the holding motors");
            telemetry.addLine("Current Left Position " + slides.getLeftPosition());
            telemetry.addLine("Current Right Position " + slides.getRightPosition());
            telemetry.addLine("Target Position " + slides.getTargetPosition());
            telemetry.update();
        }
    }
}

