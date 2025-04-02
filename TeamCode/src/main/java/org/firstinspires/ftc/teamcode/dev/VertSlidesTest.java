//package org.firstinspires.ftc.teamcode.dev;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;
//import org.firstinspires.ftc.teamcode.subsystems.VertSlidePair;
//
//@TeleOp(name = "Vert Slides Test", group = "Dev")
//public class VertSlidesTest extends LinearOpMode {
//    static final double SLIDE_POWER = 1.0;
//    @Override
//    public void runOpMode() {
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        VertSlidePair slides = new VertSlidePair(hardwareMap);
//        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            if (gamepads.isPressed(-1, "square")) {
//                slides.resetPosition();
//            }
//            if (gamepads.isPressed("right_bumper")){
//                if (slides.isActive(VertSlidePair.SlideSide.RIGHT)){
//                    slides.setPower(VertSlidePair.SlideSide.RIGHT, 0);
//                } else{
//                    slides.setPower(VertSlidePair.SlideSide.RIGHT, SLIDE_POWER);
//                }
//            }
//            if (gamepads.isPressed("left_bumper")){
//                if (slides.isActive(VertSlidePair.SlideSide.LEFT)){
//                    slides.setPower(VertSlidePair.SlideSide.LEFT, 0);
//                } else{
//                    slides.setPower(VertSlidePair.SlideSide.LEFT, SLIDE_POWER);
//                }
//            }
//            if (gamepads.isHeld("dpad_up")){
//                slides.changeTargetPosition(VertSlidePair.SlideSide.LEFT,50);
//            }
//            if (gamepads.isHeld("dpad_down")){
//                slides.changeTargetPosition(VertSlidePair.SlideSide.LEFT,-50);
//            }
//            if (gamepads.isHeld("triangle")){
//                slides.changeTargetPosition(VertSlidePair.SlideSide.RIGHT,50);
//            }
//            if (gamepads.isHeld("cross")){
//                slides.changeTargetPosition(VertSlidePair.SlideSide.RIGHT,-50);
//            }
//            //if (gamepads.isPressed(1, "cross")){
//            //slides.performCycleMove(2000, 3000);
//            //}
//
//            telemetry.addLine("Use the D-pad and up and down face buttons to control the slides.");
//            telemetry.addLine("Press ▣ to reset the slides position to 0.");
//            telemetry.addLine("Press Either Bumper to toggle the holding motors for a side");
//            telemetry.addLine(slides.toString());
//            telemetry.update();
//        }
//    }
//}
//
