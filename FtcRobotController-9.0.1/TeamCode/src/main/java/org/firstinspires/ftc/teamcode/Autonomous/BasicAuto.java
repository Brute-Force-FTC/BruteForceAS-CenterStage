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
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous
public class BasicAuto extends LinearOpMode{

    double globalAngle, power = .30, correction;

    @Override
    public void runOpMode() {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        TensorFlowUtils utils = new TensorFlowUtils(hardwareMap);



        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.addData("Status", "Initialized");
        utils.telemetryTfod(telemetry);
        telemetry.update();

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        //Check IMU is calibrated
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();
        waitForStart();
        robot.resetEncoder();
        robot.runUsingEncoder();
        int pos = utils.returnTSEPosition(opModeIsActive(), telemetry);

        robot.resetAngle(globalAngle);
        correction = robot.checkDirectionB(globalAngle, telemetry);

        //robot.encoderDriveIMU("f", globalAngle, 10, 1, opModeIsActive(), telemetry);

        //robot.encoderDrive("f", 5, 0.5, opModeIsActive(), telemetry);
        //robot.encoderDrive("b", 5, 0.5, opModeIsActive(), telemetry);
        //robot.encoderDrive("r", 5, 0.5, opModeIsActive(), telemetry); //rot left
        //robot.encoderDrive("l", 5, 0.5, opModeIsActive(), telemetry); //str right
        //robot.encoderDrive("rr", 5, 0.5, opModeIsActive(), telemetry); //turn left
        //robot.encoderDrive("rl", 5, 0.5, opModeIsActive(), telemetry); //turn right

        robot.intakeLeft.setPower(-1);
        sleep(5000);
    }
}
