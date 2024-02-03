package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utilities.BruteForceRobot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class BasicTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        robot.slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            double lsy1 = gamepad1.left_stick_y;
            double lsx1 = gamepad1.left_stick_x;
            double rsy1 = gamepad1.right_stick_y;
            double rsx1 = gamepad1.right_stick_x;
            double lsy2 = gamepad2.left_stick_y;
            double lsx2 = gamepad2.left_stick_x;
            double rsy2 = gamepad2.right_stick_y;
            double rsx2 = gamepad2.right_stick_x;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            gamepad1.right_stick_x
                    )
            );

            if (rsy2 != 0) {
                robot.slideLeft.setPower(rsy2*0.25);
                robot.slideRight.setPower(-rsy2*0.25);
            } else {
                robot.slideLeft.setPower(0);
                robot.slideRight.setPower(0);//else {
            }

            if (robot.slideLeft.getCurrentPosition() < -2000 || robot.slideRight.getCurrentPosition() > 2000) {
                robot.encoderSL(-1900);
                robot.encoderSR(1900);
            } else if (robot.slideLeft.getCurrentPosition() > 10 || robot.slideRight.getCurrentPosition() < -10) {
                robot.encoderSL(-90);
                robot.encoderSR(90 );
            }

            if (gamepad1.x) {
                robot.encoderSL(-1000);
                robot.encoderSR(1000);

            }

            if (lsy2 != 0) {
                robot.intake.setPower(lsy2*0.75);
            } else {
                robot.intake.setPower(0);
            }

            if (gamepad2.a) {
                robot.arm.setPower(-1); //up
            }

            if (gamepad2.b) {
                robot.arm.setPower(1); //down
            }

            if (gamepad2.x) {
                robot.intakeRight.setPower(-0.35); //down
            }

            if (gamepad2.y) {
                robot.intakeRight.setPower(0.1); //up
            }

            if (gamepad2.right_bumper) {
                robot.boxClaw.setPower(-1);
            }

            if(gamepad2.left_bumper) {
                robot.boxClaw.setPower(1);
            }

            if (gamepad1.right_bumper) {
                robot.paperAirplane.setPower(1);
            }

            if (gamepad1.left_bumper) {
                robot.paperAirplane.setPower(-1);
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("SL Encoder", robot.slideLeft.getCurrentPosition());
            telemetry.addData("SL Power", robot.slideLeft.getPower());
            telemetry.addData("SR Encoder", robot.slideRight.getCurrentPosition());
            telemetry.addData("SR Power", robot.slideRight.getPower());
            telemetry.update();
        }
    }
}
