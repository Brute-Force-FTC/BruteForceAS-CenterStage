package org.firstinspires.ftc.teamcode.TeleOp;

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
import org.firstinspires.ftc.teamcode.Utilities.BruteForceRobot;

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
                robot.RLAmotor.setPower(rsy2);
                robot.LLAmotor.setPower(rsy2);
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
                robot.clawServo.setPower(1);
                sleep(600);
                robot.clawServo.setPower(-1);
            }

            if (gamepad2.b) {
                robot.clawRotator.setPower(1);
                sleep(600);
                robot.clawRotator.setPower(-1);
            }

            if (gamepad2.x) {
                robot.testServo.setPower(1);
                sleep(600);
                robot.testServo.setPower(-1);
            }

            if (gamepad1.a) {
                robot.RLAmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.LLAmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.RLAmotor.setTargetPosition(2000);
                robot.LLAmotor.setTargetPosition(2000);
                robot.RLAmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.LLAmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.RLAmotor.setPower(1);
                robot.LLAmotor.setPower(1);
                while (robot.RLAmotor.isBusy()) {
                    while (robot.LLAmotor.isBusy()) {
                        ;
                    }
                }
                robot.RLAmotor.setPower(0);
                robot.LLAmotor.setPower(0);
                robot.RLAmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.LLAmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad1.b) {
                robot.RLAmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.LLAmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.RLAmotor.setTargetPosition(2000);
                robot.LLAmotor.setTargetPosition(2000);
                robot.RLAmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.LLAmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.RLAmotor.setPower(-1);
                robot.LLAmotor.setPower(1);
                while (robot.RLAmotor.isBusy()) {
                    while (robot.LLAmotor.isBusy()) {
                        ;
                    }
                }
                robot.RLAmotor.setPower(0);
                robot.LLAmotor.setPower(0);
                robot.RLAmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.LLAmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad1.x) {
                robot.RLAmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.LLAmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.RLAmotor.setTargetPosition(2000);
                robot.LLAmotor.setTargetPosition(2000);
                robot.RLAmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.LLAmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.RLAmotor.setPower(1);
                robot.LLAmotor.setPower(-1);
                while (robot.RLAmotor.isBusy()) {
                    while (robot.LLAmotor.isBusy()) {
                        ;
                    }
                }
                robot.RLAmotor.setPower(0);
                robot.LLAmotor.setPower(0);
                robot.RLAmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.LLAmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad1.y) {
                robot.RLAmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.LLAmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.RLAmotor.setTargetPosition(2000);
                robot.LLAmotor.setTargetPosition(2000);
                robot.RLAmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.LLAmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.RLAmotor.setPower(-1);
                robot.LLAmotor.setPower(-1);
                while (robot.RLAmotor.isBusy()) {
                    while (robot.LLAmotor.isBusy()) {
                        ;
                    }
                }
                robot.RLAmotor.setPower(0);
                robot.LLAmotor.setPower(0);
                robot.RLAmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.LLAmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            telemetry.addData("LLA Encoder", robot.LLAmotor.getCurrentPosition());
            telemetry.addData("RLA Encoder", robot.RLAmotor.getCurrentPosition());
            telemetry.addData("clawArm Encoder", robot.clawArm.getCurrentPosition());
            telemetry.addData("clawArm Power", robot.clawArm.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
