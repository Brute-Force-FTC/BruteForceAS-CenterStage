package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Utilities.BruteForceRobot;

@TeleOp
public class MyTeleOp extends LinearOpMode {

    double rsy1 = 0;
    double rsx1 = 0;
    double lsx1 = 0;
    double lsy1 = 0;
    double rsy2 = 0;
    double rsx2 = 0;
    double lsx2 = 0;
    double lsy2 = 0;

    public MyTeleOp() throws Exception {
        RobotLog.d("Starting TeleOp");
    }
    @Override
    public void runOpMode() throws InterruptedException {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        robot.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        Thread slideThread = new slideThread();
        Thread intakeThread = new intakeThread();

        waitForStart();

        slideThread.start();
        intakeThread.start();

        if (isStopRequested()) return;

        try {
            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                robot.frontLeft.setPower(frontLeftPower);
                robot.backLeft.setPower(backLeftPower);
                robot.frontRight.setPower(frontRightPower);
                robot.backRight.setPower(backRightPower);

                telemetry.addData("frontLeft Power", robot.frontLeft.getPower());
                telemetry.addData("frontRight Power", robot.frontRight.getPower());
                telemetry.addData("backLeft Power", robot.backLeft.getPower());
                telemetry.addData("backRight Power", robot.backRight.getPower());
                telemetry.update();
            }

            idle();
        }
        catch(Exception e) {RobotLog.d(e.getMessage());}
        slideThread.interrupt();
        intakeThread.interrupt();
    }

    private class slideThread extends Thread {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);

        public slideThread() {
            this.setName("slideThread");
            RobotLog.d("%s", this.getName());
        }

        @Override
        public void run() {
            try {
                while (!isInterrupted()) {
                    //Slide Kit Thread

                    lsy1 = gamepad1.left_stick_y;
                    lsx1 = gamepad1.left_stick_x;
                    rsy1 = gamepad1.right_stick_y;
                    rsx1 = gamepad1.right_stick_x;
                    lsy2 = gamepad2.left_stick_y;
                    lsx2 = gamepad2.left_stick_x;
                    rsy2 = gamepad2.right_stick_y;
                    rsx2 = gamepad2.right_stick_x;

                    if (rsy2 != 0) {
                        robot.slideLeft.setPower(rsy2);
                        robot.slideRight.setPower(-rsy2);
                    } //else {
                    //if (robot.RLAmotor.getCurrentPosition() != robot.LLAmotor.getCurrentPosition()) {
                    //  robot.LLAmotor.setPower((robot.LLAmotor.getCurrentPosition() - robot.RLAmotor.getCurrentPosition())*0.001);
                    //}
                    //}

                    /*PID loop
                    //ideally error must reach 0 in the system
                    double kP = 0.1;
                    double kD = 0.01;
                    double kI = 0.001;
                    double reference = robot.LLAmotor.getCurrentPosition();
                    double integralSum = 0;
                    double lastError = 0;
                    ElapsedTime timer = new ElapsedTime();
                    double pos, error, derivative, out;

                    while (robot.RLAmotor.getCurrentPosition() != robot.LLAmotor.getCurrentPosition()) {
                        pos = robot.RLAmotor.getCurrentPosition();
                        error = robot.LLAmotor.getCurrentPosition() - robot.RLAmotor.getCurrentPosition();
                        derivative = (error - lastError) / timer.seconds();
                        integralSum = integralSum + (error*timer.seconds());
                        out = (kP*error) + (kI*integralSum) + (kD*derivative);
                        robot.RLAmotor.setPower(out);
                        lastError = error;
                        timer.reset();
                    }
                    //end PID loop*/

                    //if ((robot.RLAmotor.getCurrentPosition() >= 3400) || (robot.LLAmotor.getCurrentPosition() >= 3400)) {
                    //  robot.encoderRLA(3200);
                    //  robot.encoderLLA(3200);
                    //}
                    idle();
                }
            }
            catch (Exception e) {e.printStackTrace();}
        }
    }

    private class intakeThread extends Thread {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);

        public intakeThread() {
            this.setName("intakeThread");
            RobotLog.d("%s", this.getName());
        }

        @Override
        public void run() {
            try {
                while (!isInterrupted()) {
                    //Intake Thread

                    lsy1 = gamepad1.left_stick_y;
                    lsx1 = gamepad1.left_stick_x;
                    rsy1 = gamepad1.right_stick_y;
                    rsx1 = gamepad1.right_stick_x;
                    lsy2 = gamepad2.left_stick_y;
                    lsx2 = gamepad2.left_stick_x;
                    rsy2 = gamepad2.right_stick_y;
                    rsx2 = gamepad2.right_stick_x;

                    if (lsy2 != 0) {
                        robot.intake.setPower(lsy2);
                    } else {
                        //robot.clawArm.setPower(0.01);
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

                    if (gamepad2.left_bumper) {
                        robot.boxClaw.setPower(1);
                    }

                    idle();
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}