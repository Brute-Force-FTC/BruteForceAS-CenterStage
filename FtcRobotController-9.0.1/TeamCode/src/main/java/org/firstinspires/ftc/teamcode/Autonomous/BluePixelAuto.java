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
public class BluePixelAuto extends LinearOpMode{

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
        double open = 1;
        double close = -1;

        //close claw

        pos = 2;

        if (pos == 1) {
            //Move forward towards spike marks
            robot.encoderDrive("f", 12, 0.5, opModeIsActive(), telemetry);

            //Turn left to face the left spike mark
            robot.encoderDrive("rl", 5, 0.5, opModeIsActive(), telemetry);

            //Open claw

            //Lift arm up

            //Close claw

            //Move forward towards the backdrop
            robot.encoderDrive("f", 5, 0.5, opModeIsActive(), telemetry);

            //Turn left so robot is facing backdrop
            robot.encoderDrive("rl", 3, 0.5, opModeIsActive(), telemetry);

            //Move forward so claw is on the backdrop
            robot.encoderDrive("f", 3, 0.5, opModeIsActive(), telemetry);

            //Open claw

            //Move backward
            robot.encoderDrive("b", 5, 0.5, opModeIsActive(), telemetry);

            //Close claw

            //Move arm down

            //Strafe right to park
            robot.encoderDrive("r", 10, 0.5, opModeIsActive(), telemetry);

            //Move forward to get into the parking spot
            robot.encoderDrive("f", 5, 0.5, opModeIsActive(), telemetry);
        } else if (pos == 2) {
            robot.encoderDrive("f", 20, 0.5, opModeIsActive(), telemetry);
            robot.encoderDrive("rl", 10, 0.5, opModeIsActive(), telemetry);
            robot.encoderDrive("f", 75, 0.5, opModeIsActive(), telemetry);
        }
        //robot.encoderDrive("r", 1, 0.5, opModeIsActive(), telemetry);
        sleep(700);
    }
}
