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

@TeleOp
public class BasicTeleOp extends LinearOpMode{
    private Gyroscope imu;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Blinker expansion_Hub_2;
    private Blinker controlHub;
    private HardwareDevice webcam_1;
    private CRServo clawServo;
    private DcMotor RLAmotor;
    private DcMotor LLAmotor;
    private DcMotor clawArm;
    private DcMotor testMotor;



    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        controlHub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        webcam_1 = hardwareMap.get(HardwareDevice.class, "Webcam 1");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        clawServo = hardwareMap.get(CRServo.class, "clawServo");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        RLAmotor = hardwareMap.get(DcMotor.class, "RLAmotor");
        LLAmotor = hardwareMap.get(DcMotor.class, "LLAmotor");
        clawArm = hardwareMap.get(DcMotor.class, "clawArm");
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");



        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

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
                frontLeft.setPower(lsy1);
                frontRight.setPower(-lsy1);
                backLeft.setPower(lsy1);
                backRight.setPower(-lsy1);
            } else if (lsx1 != 0) {
                frontLeft.setPower(-lsx1);
                frontRight.setPower(-lsx1);
                backLeft.setPower(lsx1);
                backRight.setPower(lsx1);
            } else if (rsx1 != 0) {
                frontLeft.setPower(rsx1);
                frontRight.setPower(rsx1);
                backLeft.setPower(rsx1);
                backRight.setPower(rsx1);
                //} else if (rsy2 != 0) {
                //RLAmotor.setPower(-rsy2);
                //LLAmotor.setPower(rsy2);
            } else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                RLAmotor.setPower(0);
                LLAmotor.setPower(0);
            }

            telemetry.addData("LSY1:", lsy1);
            telemetry.addData("LSX1:", lsx1);
            telemetry.addData("RSY1:", rsy1);
            telemetry.addData("RSX1:", rsx1);
            telemetry.addData("LSY2:", lsy2);
            telemetry.addData("LSX2:", lsx2);
            telemetry.addData("RSY2:", rsy2);
            telemetry.addData("RSX2:", rsx2);
            telemetry.addData("frontLeft Power", frontLeft.getPower());
            telemetry.addData("frontRight Power", frontRight.getPower());
            telemetry.addData("backLeft Power", backLeft.getPower());
            telemetry.addData("backRight Power", backRight.getPower());
            telemetry.addData("LLA Encoder", LLAmotor.getCurrentPosition());
            telemetry.addData("RLA Encoder", RLAmotor.getCurrentPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
