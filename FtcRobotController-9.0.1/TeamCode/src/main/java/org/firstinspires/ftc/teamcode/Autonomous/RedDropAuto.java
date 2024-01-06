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
        int pos = utils.returnTSEPosition(opModeIsActive(), telemetry);
        double open = 1;
        double close = -1;

        //close claw
        //robot.clawServo.setPower(close);

        pos = 1;

        if (pos == 1) {
            //Move backwards to go towards the spike marks
            robot.encoderDrive("f", 3 , 0.5, opModeIsActive(), telemetry);

            //Move arm up

            //Turn all the way so the robot is facing the spike marks
            //robot.encoderDrive("rr", 20, 0.5, opModeIsActive(), telemetry);

            //Turn left to face the left spike mark
            //robot.encoderDrive("rr", 10, 0.5, opModeIsActive(), telemetry);

            //Open claw
            //robot.clawServo.setPower(open);

            //Move forward to avoid hitting the truss
            //robot.encoderDrive("f", 5 , 0.5, opModeIsActive(), telemetry);

            //Rotate left so robot is facing backdrop
            //robot.encoderDrive("rl", 5, 0.5, opModeIsActive(), telemetry);

            //Move forward to park
            robot.encoderDrive("r", 40, 0.5, opModeIsActive(), telemetry);
        } else if (pos == 2) {
            //Move backwards to go towards the spike marks
            robot.encoderDrive("b", 10 , 0.5, opModeIsActive(), telemetry);

            //Move arm up

            //Turn all the way so the robot is facing the spike marks
            robot.encoderDrive("rr", 20, 0.5, opModeIsActive(), telemetry);

            //Move forward to spike mark
            robot.encoderDrive("f", 5, 0.5, opModeIsActive(), telemetry);

            //Open claw

            //Rotate left so robot is facing backdrop
            robot.encoderDrive("rl", 10, 0.5, opModeIsActive(), telemetry);

            //Move forward to park
            robot.encoderDrive("f", 50, 0.5, opModeIsActive(), telemetry);
        } else if (pos == 3) {
            //Move backwards to go towards the spike marks
            robot.encoderDrive("b", 10 , 0.5, opModeIsActive(), telemetry);

            //Move arm up

            //Turn all the way so the robot is facing the spike marks
            robot.encoderDrive("rr", 20, 0.5, opModeIsActive(), telemetry);

            //Turn right to face the right spike mark
            robot.encoderDrive("rr", 5, 0.5, opModeIsActive(), telemetry);

            //Open claw

            //Move backward to avoid hitting the truss
            robot.encoderDrive("b", 5 , 0.5, opModeIsActive(), telemetry);

            //Rotate left so robot is facing backdrop
            robot.encoderDrive("rl", 15, 0.5, opModeIsActive(), telemetry);

            //Move forward to park
            robot.encoderDrive("f", 50, 0.5, opModeIsActive(), telemetry);
        }
    }
}
