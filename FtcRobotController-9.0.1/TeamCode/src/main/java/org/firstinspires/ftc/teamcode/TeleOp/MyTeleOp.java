package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Utilities.BruteForceRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@TeleOp
public class MyTeleOp extends LinearOpMode{

    @Override
    public void runOpMode() {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);



        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.resetEncoder();
        robot.runUsingEncoder();

        while (opModeIsActive()) {
            double lsy1 = this.gamepad1.left_stick_y;
            double lsx1 = this.gamepad1.left_stick_x;
            double rsy1 = this.gamepad1.right_stick_y;
            double rsx1 = this.gamepad1.right_stick_x;
            double lsy2 = this.gamepad2.left_stick_y;
            double lsx2 = this.gamepad2.left_stick_x;
            double rsy2 = this.gamepad2.right_stick_y;
            double rsx2 = this.gamepad2.right_stick_x;
            if (lsy1 != 0) {
                robot.moveVertical(lsy1);
            } else if (lsx1 != 0) {
                robot.moveHorizontal(lsx1);
            } else if (rsx1 != 0) {
                robot.rotate(rsx1);
            } else {
                robot.stopMoving();
            }

            if (rsy2 != 0) {
            } //else {
                //if (robot.RLAmotor.getCurrentPosition() != robot.LLAmotor.getCurrentPosition()) {
                    //robot.LLAmotor.setPower((robot.LLAmotor.getCurrentPosition() - robot.RLAmotor.getCurrentPosition())*0.001);
                //}
            //}

            //if ((robot.RLAmotor.getCurrentPosition() >= 3400) || (robot.LLAmotor.getCurrentPosition() >= 3400)) {
                //robot.encoderRLA(3200);
                //robot.encoderLLA(3200);
            //}

            if (gamepad2.a) {
            }

            if (gamepad2.b) {
            }

            if (gamepad2.x) {
            }

            //if (robot.RLAmotor.getCurrentPosition() != robot.LLAmotor.getCurrentPosition()) {
                //robot.encoderLLA(robot.RLAmotor.getCurrentPosition());
            //}

            //if (robot.RLAmotor.getCurrentPosition() != robot.LLAmotor.getCurrentPosition()) {
                //robot.RLAmotor.setPower((robot.RLAmotor.getCurrentPosition() - robot.LLAmotor.getCurrentPosition())*0.001);
            //}

            telemetry.addData("LSY1:", lsy1);
            telemetry.addData("LSX1:", lsx1);
            telemetry.addData("RSY1:", rsy1);
            telemetry.addData("RSX1:", rsx1);
            telemetry.addData("LSY2:", lsy2);
            telemetry.addData("LSX2:", lsx2);
            telemetry.addData("RSY2:", rsy2);
            telemetry.addData("RSX2:", rsx2);
            telemetry.addData("frontLeft Power", robot.frontLeft.getPower());
            telemetry.addData("frontRight Power", robot.frontRight.getPower());
            telemetry.addData("backLeft Power", robot.backLeft.getPower());
            telemetry.addData("backRight Power", robot.backRight.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
