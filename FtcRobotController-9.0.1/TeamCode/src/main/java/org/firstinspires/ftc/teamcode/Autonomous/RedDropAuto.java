package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.Utilities.TensorFlowUtils;

@Autonomous
public class RedDropAuto extends LinearOpMode{

    @Override
    public void runOpMode() {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        TensorFlowUtils utils = new TensorFlowUtils(hardwareMap);



        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.addData("Status", "Initialized");
        utils.telemetryTfod(telemetry);
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.resetEncoder();
        robot.runUsingEncoder();
        int pos = utils.returnTSEPositionRDA(opModeIsActive(), telemetry);
        telemetry.addData("pos", pos);
        telemetry.update();

        int c = -1;
        int o = 1;

        if (pos == 1) {
            //Move forward right before all 3 spike marks
            robot.encoderDrive("f", 22.5, 0.5, opModeIsActive(), telemetry);

            //Turn left so that the ramp is facing the left spike mark
            robot.encoderDrive("rl",  12, 0.5, opModeIsActive(), telemetry);

            //Move forward right before all 3 spike marks
            robot.encoderDrive("f", 1.25, 0.5, opModeIsActive(), telemetry);

            //Open the intake so the purple pixel drops
            sleep(500);
            robot.intakeLeft.setPower(o);
            sleep(500);

            //Back up to go to the parking area
            robot.encoderDrive("b", 5, 0.5, opModeIsActive(), telemetry);

            //Close the intake
            sleep(500);
            robot.intakeLeft.setPower(c);

            //Rotate left so that the robot is facing the backdrop
            robot.encoderDrive("rr", 32, 0.5, opModeIsActive(), telemetry);

            //Move forward so the ramp is inside of the parking area
            robot.encoderDrive("f", 35, 0.5, opModeIsActive(), telemetry);

            //Open the intake
            sleep(500);
            robot.intakeLeft.setPower(o);
            sleep(500);

            //Spin the intake
            sleep(500);
            robot.intake.setPower(1);
            sleep(2000);
            robot.intake.setPower(0);

            //Move the robot backward
            robot.encoderDrive("b", 10, 0.5, opModeIsActive(), telemetry);

            //Strafe to the left
            robot.encoderDrive("r", 15 , 0.5, opModeIsActive(), telemetry);

            //Move forward to park
            robot.encoderDrive("f", 20, 0.5, opModeIsActive(), telemetry);
        } else if (pos == 2) {
            //Move forward until the ramp is on the middle spike mark
            robot.encoderDrive("f", 30, 0.5, opModeIsActive(), telemetry);

            //Open the intake so the purple pixel drops
            sleep(500);
            robot.intakeLeft.setPower(o);
            sleep(500);

            //Back up to go to the parking area
            robot.encoderDrive("b", 5, 0.5, opModeIsActive(), telemetry);

            //Close the intake
            sleep(500);
            robot.intakeLeft.setPower(c);

            //Rotate left so that the robot is facing the backdrop
            robot.encoderDrive("rr", 20, 0.5, opModeIsActive(), telemetry);

            //Move forward so the ramp is inside of the parking area
            robot.encoderDrive("f", 27.5, 0.5, opModeIsActive(), telemetry);

            //Open the intake
            sleep(500);
            robot.intakeLeft.setPower(o);
            sleep(500);

            //Spin the intake
            sleep(500);
            robot.intake.setPower(1);
            sleep(2000);
            robot.intake.setPower(0);

            //Move the robot backward
            robot.encoderDrive("b", 10, 0.5, opModeIsActive(), telemetry);

            //Strafe to the left
            robot.encoderDrive("r", 15 , 0.5, opModeIsActive(), telemetry);

            //Move forward to park
            robot.encoderDrive("f", 20, 0.5, opModeIsActive(), telemetry);
        } else if (pos == 0) {
            //Move forward right before all 3 spike marks
            robot.encoderDrive("f", 22.5, 0.5, opModeIsActive(), telemetry);

            //Turn right so that the ramp is facing the left spike mark
            robot.encoderDrive("rr",  12, 0.5, opModeIsActive(), telemetry);

            //Move forward right before all 3 spike marks
            robot.encoderDrive("f", 1.25, 0.5, opModeIsActive(), telemetry);

            //Open the intake so the purple pixel drops
            sleep(500);
            robot.intakeLeft.setPower(o);
            sleep(500);

            //Back up to go to the parking area
            robot.encoderDrive("b", 5, 0.5, opModeIsActive(), telemetry);

            //Close the intake
            sleep(500);
            robot.intakeLeft.setPower(c);

            //Rotate left so that the robot is facing the backdrop
            robot.encoderDrive("rr", 8, 0.5, opModeIsActive(), telemetry);

            //Move forward so the ramp is inside of the parking area
            robot.encoderDrive("f", 35, 0.5, opModeIsActive(), telemetry);

            //Open the intake
            sleep(500);
            robot.intakeLeft.setPower(o);
            sleep(500);

            //Spin the intake
            sleep(500);
            robot.intake.setPower(1);
            sleep(2000);
            robot.intake.setPower(0);

            //Move the robot backward
            robot.encoderDrive("b", 10, 0.5, opModeIsActive(), telemetry);

            //Strafe to the left
            robot.encoderDrive("r", 15 , 0.5, opModeIsActive(), telemetry);

            //Move forward to park
            robot.encoderDrive("f", 20, 0.5, opModeIsActive(), telemetry);
        }
    }
}
