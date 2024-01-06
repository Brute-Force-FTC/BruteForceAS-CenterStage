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
public class RedPixelAuto extends LinearOpMode{

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
        int pos = utils.returnTSEPosition(opModeIsActive(), telemetry);

        //robot.encoderDrive("r", 1, 0.5, opModeIsActive(), telemetry);

        pos = 1;

        if (pos == 1) {
            robot.encoderDrive("f", 20, 0.5, opModeIsActive(), telemetry);
            robot.encoderDrive("rr", 10, 0.5, opModeIsActive(), telemetry);
            robot.encoderDrive("f", 75, 0.5, opModeIsActive(), telemetry);
        }
    }
}
