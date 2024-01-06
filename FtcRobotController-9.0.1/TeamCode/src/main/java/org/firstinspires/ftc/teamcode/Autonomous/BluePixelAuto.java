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
        sleep(500);
        int pos = utils.returnTSEPositionBPA(opModeIsActive(), telemetry);

        if (pos == 1) {
            //Move forward right before all 3 spike marks
            robot.encoderDrive("f", 5 , 0.5, opModeIsActive(), telemetry);

            //Turn left so that the ramp is facing the left spike mark
            robot.encoderDrive("rl",  10, 0.5, opModeIsActive(), telemetry);

            //Open the intake so the purple pixel drops
            sleep(500);
            robot.intakeLeft.setPower(-1);
            sleep(500);

            //Back up to go to the parking area
            robot.encoderDrive("b", 15, 0.5, opModeIsActive(), telemetry);

            //Close the intake
            sleep(500);
            robot.intakeLeft.setPower(1);

            //Rotate left so that the robot is facing the backdrop
            robot.encoderDrive("rl", 10, 0.5, opModeIsActive(), telemetry);

            //Move forward so the ramp is inside of the parking area
            robot.encoderDrive("f", 50, 0.5, opModeIsActive(), telemetry);

            //Open the intake
            sleep(500);
            robot.intakeLeft.setPower(-1);
            sleep(500);

            //Spin the intake
            sleep(500);
            robot.intake.setPower(1);
            sleep(1000);
            robot.intake.setPower(0);

            //Move the robot backward so a little is still in parking
            robot.encoderDrive("b", 3, 0.5, opModeIsActive(), telemetry);
        } else if (pos == 2) {
            //Move forward until the ramp is on the middle spike mark
            robot.encoderDrive("f", 12, 0.5, opModeIsActive(), telemetry);

            //Open the intake so the purple pixel drops
            sleep(500);
            robot.intakeLeft.setPower(-1);
            sleep(500);

            //Back up to go to the parking area
            robot.encoderDrive("b", 10, 0.5, opModeIsActive(), telemetry);

            //Close the intake
            sleep(500);
            robot.intakeLeft.setPower(1);

            //Rotate left so that the robot is facing the backdrop
            robot.encoderDrive("rl", 20, 0.5, opModeIsActive(), telemetry);

            //Move forward so the ramp is inside of the parking area
            robot.encoderDrive("f", 50, 0.5, opModeIsActive(), telemetry);

            //Open the intake
            sleep(500);
            robot.intakeLeft.setPower(-1);
            sleep(500);

            //Spin the intake
            sleep(500);
            robot.intake.setPower(1);
            sleep(1000);
            robot.intake.setPower(0);

            //Move the robot backward so a little is still in parking
            robot.encoderDrive("b", 3, 0.5, opModeIsActive(), telemetry);
        } else if (pos == 3) {
            //Move forward right before all 3 spike marks
            robot.encoderDrive("f", 5 , 0.5, opModeIsActive(), telemetry);

            //Turn right so that the ramp is facing the right spike mark
            robot.encoderDrive("rr",  10, 0.5, opModeIsActive(), telemetry);

            //Open the intake so the purple pixel drops
            sleep(500);
            robot.intakeLeft.setPower(-1);
            sleep(500);

            //Back up to go to the parking area
            robot.encoderDrive("b", 3, 0.5, opModeIsActive(), telemetry);

            //Close the intake
            sleep(500);
            robot.intakeLeft.setPower(1);

            //Rotate left so the robot is facing straight
            robot.encoderDrive("rl", 10, 0.5, opModeIsActive(), telemetry);

            //Move backward so the robot is aligned to the parking area
            robot.encoderDrive("b", 8, 0.5, opModeIsActive(), telemetry);

            //Rotate left so that the robot is facing the backdrop
            robot.encoderDrive("rl", 20, 0.5, opModeIsActive(), telemetry);

            //Move forward so the ramp is inside of the parking area
            robot.encoderDrive("f", 50, 0.5, opModeIsActive(), telemetry);

            //Open the intake
            sleep(500);
            robot.intakeLeft.setPower(-1);
            sleep(500);

            //Spin the intake
            sleep(500);
            robot.intake.setPower(1);
            sleep(1000);
            robot.intake.setPower(0);

            //Move the robot backward so a little is still in the parking space
            robot.encoderDrive("b", 3, 0.5, opModeIsActive(), telemetry);
        }
    }
}
