/*package org.firstinspires.ftc.teamcode.FTC2023;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Stack;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


public class Test_init {

    //Drive Motors
    public DcMotor Lfront;
    public DcMotor Rfront;
    public DcMotor Lback;
    public DcMotor Rback;
    public DcMotor Slide;
    public DcMotor SSlide;
    //public DcMotor Intake;
    //public DcMotor Slide;

    //Drive
    public double Lfronttmp = 0;
    public double Lbacktmp = 0;
    public double Rfronttmp = 0;
    public double Rbacktmp = 0;

    public double Lfrontforward = 0;
    public double Lbackforward = 0;
    public double Rfrontforward = 0;
    public double Rbackforward = 0;

    //imu
    BNO055IMU imu;

    //servo
    public Servo Intake;
    //public Servo SSlide;
    //public Servo Cap;

    public HardwareMap _hw;
    public ColorSensor _cs;
    public void init(HardwareMap hw) {
        _hw = hw;

        
        // Rback = _hw.dcMotor.get("Lfront");
        // Lback = _hw.dcMotor.get("Rfront");
        // Rfront = _hw.dcMotor.get("Lback");
        // Lfront = _hw.dcMotor.get("Rback");


        Lfront = _hw.dcMotor.get("Lfront");
        Rfront = _hw.dcMotor.get("Rfront");
        Lback = _hw.dcMotor.get("Lback");
        Rback = _hw.dcMotor.get("Rback");
        //Duck = _hw.dcMotor.get("Duck");
        //Intake = _hw.dcMotor.get("Intake");
        Slide = _hw.dcMotor.get("Slide");
        SSlide = _hw.dcMotor.get("SSlide");
        //Slide2 = _hw.dcMotor.get("Slide2");
        _cs=_hw.get(ColorSensor.class, "CS");
        imu = _hw.get(BNO055IMU.class, "imu");

        Intake = _hw.servo.get("Intake");
        //SSlide = _hw.servo.get("SSlide");
        //Cap = _hw.servo.get("Cap");
        
        //SlideL.setDirection(Servo.Direction.REVERSE);
        //Cap.setDirection(Servo.Direction.REVERSE);

        SSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        Rfront.setDirection(DcMotorSimple.Direction.REVERSE);

        Rback.setDirection(DcMotorSimple.Direction.REVERSE);

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        

    }

    public void runModeSet(String mode) {
        if (mode == "position") {
            Lfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Rfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Rback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //Slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (mode == "reset") {
            Lfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Lback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Rfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Rback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //Slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else if (mode == "tele") {
            Lfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Lback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Rfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Rback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //Slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (mode == "encoder") {
            Lfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Lback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Rfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Rback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //Slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
*/