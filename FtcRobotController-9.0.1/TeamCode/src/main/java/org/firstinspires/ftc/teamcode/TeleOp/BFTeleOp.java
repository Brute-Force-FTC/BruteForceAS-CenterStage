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
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utilities.BruteForceRobot;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Func;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.io.PrintStream;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.BruteForceRobot;

@TeleOp
public class BFTeleOp extends LinearOpMode{

    double rsy1 = 0;
    double rsx1 = 0;
    double lsx1 = 0;
    double lsy1 = 0;
    double rsy2 = 0;
    double rsx2 = 0;
    double lsx2 = 0;
    double lsy2 = 0;

    double globalAngle, power = .30, correction;

    public BFTeleOp() throws Exception {
        RobotLog.d("Starting TeleOp");
    }
    
    @Override
    public void runOpMode() throws InterruptedException {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        robot.resetEncoder();
        robot.runUsingEncoder();

        Thread slideThread = new slideThread();
        Thread intakeThread = new intakeThread();
        Thread airplane = new airplane();

        waitForStart();

        slideThread.start();
        intakeThread.start();
        airplane.start();

        try {
            while (opModeIsActive()) {
                //Main Thread

                lsy1 = this.gamepad1.left_stick_y;
                lsx1 = this.gamepad1.left_stick_x;
                rsy1 = this.gamepad1.right_stick_y;
                rsx1 = this.gamepad1.right_stick_x;
                lsy2 = this.gamepad2.left_stick_y;
                lsx2 = this.gamepad2.left_stick_x;
                rsy2 = this.gamepad2.right_stick_y;
                rsx2 = this.gamepad2.right_stick_x;
                double correction = 0;

                /*if (lsy1 != 0) {
                    robot.moveVertical(lsy1*0.5);
                } else if (lsx1 != 0) {
                    robot.moveHorizontal(lsx1*0.5);
                } else if (rsx1 != 0) {
                    robot.rotate(rsx1*0.5);
                } else {
                    robot.stopMoving();
                }*/

                while (lsy1 < 0) {
                    robot.resetAngle(globalAngle);
                    correction = robot.checkDirectionF(globalAngle, telemetry);
                    //check if it works
                }

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
                telemetry.addData("SL Encoder", robot.slideLeft.getCurrentPosition());
                telemetry.addData("SL Power", robot.slideLeft.getPower());
                telemetry.addData("SR Encoder", robot.slideRight.getCurrentPosition());
                telemetry.addData("SR Power", robot.slideRight.getPower());
                telemetry.addData("Intake Power", robot.intake.getPower());
                telemetry.addData("Status", "Running");
                telemetry.update();

                idle();
            }
        }
        catch(Exception e) {RobotLog.d(e.getMessage());}

        slideThread.interrupt();
        intakeThread.interrupt();
        airplane.interrupt();
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
                        robot.slideLeft.setPower(-rsy2);
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
                    idle();
                }
            }
            catch (Exception e) {e.printStackTrace();}
        }
    }

    private class airplane extends Thread {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);

        public airplane() {
            this.setName("airplane");
            RobotLog.d("%s", this.getName());
        }

        @Override
        public void run() {
            try {
                while(!isInterrupted()) {
                    //Paper Airplane Thread

                    lsy1 = gamepad1.left_stick_y;
                    lsx1 = gamepad1.left_stick_x;
                    rsy1 = gamepad1.right_stick_y;
                    rsx1 = gamepad1.right_stick_x;
                    lsy2 = gamepad2.left_stick_y;
                    lsx2 = gamepad2.left_stick_x;
                    rsy2 = gamepad2.right_stick_y;
                    rsx2 = gamepad2.right_stick_x;

                    if (gamepad2.a) {
                        robot.intakeLeft.setPower(-1); //up
                    }

                    if (gamepad2.b) {
                        robot.intakeLeft.setPower(1); //down
                    }

                    if (gamepad2.x) {
                        robot.intakeRight.setPower(-1); //up
                    }

                    if (gamepad2.y) {
                        robot.intakeRight.setPower(1); //down
                    }

                    if (gamepad2.right_bumper) {
                        robot.paperAirplane.setPower(-1);
                    }

                    if(gamepad2.left_bumper) {
                        robot.paperAirplane.setPower(1);
                    }

                    idle();
                }
            }
            catch (Exception e) {e.printStackTrace();}
        }
    }
}
