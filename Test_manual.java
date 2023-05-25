package org.firstinspires.ftc.teamcode.FTC2023;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;

import java.util.Locale;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="FTC2023_Test_Manual",group="FTC 2023")
public class Test_manual extends OpMode {

    Test_init robot = new Test_init();

    double speedmodifier = 1;
    boolean kaimove=false;
    boolean gamepad_b_prev = false;
    boolean gamepad_b_curr = false;
    double qwe=0.4;
    double duck_speed=0;
    double intake_speed=0;

    double slidel_mid=0.5;
    double slider_mid=0.11;
    int intake_curr=1;
    Integer an_integer;
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.Lfront.setPower(0);
        robot.Rfront.setPower(0);
        robot.Lback.setPower(0);
        robot.Rback.setPower(0);
        robot.Slide.setPower(0);
        robot.SSlide.setPower(0);
        //robot.Duck.setPower(0);
        //robot.Intake.setPower(0);
        //robot.Slide.setPower(0);
        robot.Intake.scaleRange(0,1);
        //robot.SSlide.scaleRange(0,1);
        robot.Intake.setPosition(0.5);
        //robot.SSlide.setPosition(0.4);
        //robot.Cap.setPosition(0);
        //robot.runModeSet("reset");
        robot.runModeSet("encoder");
        robot.Slide.setTargetPosition(0);
        //robot.Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.SSlide.setTargetPosition(0);
        //robot.Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.SSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.SSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }
    //sleep for n idk
    public void sleep_(int n){
        int c=2357;
        for(int i=0;i<n;i++) c=c*234*c+35;
    }
    public void biggo (int position, double power) {
        if(robot.Slide.getTargetPosition()!=position) robot.Slide.setTargetPosition(position);
        robot.Slide.setPower(power);
        //robot.Slide2.setTargetPosition(position);
        //robot.Slide2.setPower(power);        
    }
    public void smallgo (int position, double power) {
        if(robot.SSlide.getTargetPosition()!=position) robot.SSlide.setTargetPosition(position);
        //robot.SSlide.setPower(power);
        //robot.Slide2.setTargetPosition(position);
        //robot.Slide2.setPower(power);        
    }

    @Override
    public void loop() {
        
        //int red=(robot._cs.red());
        //int blue =(robot._cs.blue());
        //int green=(robot._cs.green());
        //telemetry.addLine(Integer.toString(red)+" "+Integer.toString(green)+" "+Integer.toString(blue));

        //drive
        double leftstickx = 0;
        double leftsticky = 0;
        double rightstickx = 0;
        double wheelpower;
        double stickangleradians;
        double rightX;
        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double dpadpower = .2;
        double dpadturningpower = .4;
        int liftdirection = 1;
        //biggo(2500,0.8);
        Integer currpos=robot.SSlide.getCurrentPosition();
        telemetry.addLine(currpos.toString());
        if(gamepad2.dpad_up) {//posi is go down
            robot.Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
             robot.Slide.setPower(1);
             //robot.Slide2.setPower(-1);
             telemetry.addLine("+");
        }else if(gamepad2.dpad_down) {
            robot.Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
             robot.Slide.setPower(-1);
             //robot.Slide2.setPower(1);
             telemetry.addLine("-");
        }else if(robot.Slide.getMode()==DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            robot.Slide.setPower(0);
            //robot.Slide2.setPower(0);
            telemetry.addLine("no");
        }
        if (gamepad2.left_bumper){
            //if(robot.Slide.getMode()!=DcMotor.RunMode.RUN_TO_POSITION) robot.Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //uppest
            robot.Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.SSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.SSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            biggo(7200+3600,1);
            smallgo(-3000,1);
            //robot.SSlide.setPosition(0);
        }else if (gamepad2.right_bumper){
            //downest
            robot.Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.SSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.SSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            biggo(0,0.8);
            smallgo(0,0.3);
            //robot.SSlide.setPosition(1);
        }else{
            //robot.Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double d=0.47;
        if(gamepad2.x) {//ACTION 0
            intake_curr=0;
             //robot.Intake.setPosition(0.5-d);
             //if(intake_curr==0) intake_curr=1;
             //else intake_curr=0;
        }else if(gamepad2.y) {//ACTION 2
            intake_curr=2;
             //robot.Intake.setPosition(0.5+d);
             //if(intake_curr==2) intake_curr=1;
             //else intake_curr=2;
        }else{//ACTION 2
            //robot.Intake.setPosition(0.5);
        }
        if(intake_curr==0){
            robot.Intake.setPosition(0.5-d);
        }else if(intake_curr==1){
            robot.Intake.setPosition(0.5);
        }else if(intake_curr==2){
            robot.Intake.setPosition(0.5+d);
        }
        if(gamepad2.b) {
            robot.SSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.SSlide.setPower(-1);
        }else if(gamepad2.a) {
            robot.SSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.SSlide.setPower(0.7);
        }else if(robot.SSlide.getMode()==DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            robot.SSlide.setPower(0);
        }
        //if(gamepad1.left_stick_button) telemetry.addLine("have");
        //else telemetry.addLine("no");
        if (gamepad1.left_stick_button) {
            speedmodifier = 1;
        }
        if (gamepad1.right_stick_button) {
            speedmodifier = .5;
        }

        if (gamepad1.dpad_up) {
            leftsticky = dpadpower;
        } else if (gamepad1.dpad_right) {
            leftstickx = dpadturningpower;
        } else if (gamepad1.dpad_down) {
            leftsticky = -dpadpower;
        } else if (gamepad1.dpad_left) {
            leftstickx = -dpadturningpower;
        } else {
            leftstickx = gamepad1.left_stick_x;
            leftsticky = -gamepad1.left_stick_y;
            rightstickx = gamepad1.right_stick_x * speedmodifier;
        }


        if (Math.abs(leftsticky) <= .15) {
            leftsticky = 0;
        }
        wheelpower = Math.hypot(leftstickx, leftsticky);
        stickangleradians = Math.atan2(leftsticky, leftstickx);

        stickangleradians = stickangleradians - Math.PI / 4; //adjust by 45 degrees

        rightX = rightstickx * .8;
        leftfrontpower = wheelpower * Math.cos(stickangleradians) + rightX;
        rightfrontpower = wheelpower * Math.sin(stickangleradians) - rightX;
        leftrearpower = wheelpower * Math.sin(stickangleradians) + rightX;
        rightrearpower = wheelpower * Math.cos(stickangleradians) - rightX;

        //leftfrontpower = 1;
        //rightfrontpower = 1;
        //leftrearpower = 1;
        //rightrearpower = 1;
        //telemetry.addLine(Double.toString(leftfrontpower));
        robot.Lfront.setPower(leftfrontpower);
        robot.Rfront.setPower(rightfrontpower);
        robot.Lback.setPower(leftrearpower);
        robot.Rback.setPower(rightrearpower);

        // Slide Servo
        // if (gamepad1.a) {
        //     robot.SlideL.setPosition(0.4);
        //     //robot.SlideR.setPosition(90);
        // }
        // else if (gamepad1.b) {
        //     robot.SlideL.setPosition(0);
        //     //robot.SlideR.setPosition(0);

        // One Button Slide Servo
        if (gamepad2.dpad_up) {
            //robot.SlideL.setPosition(0.9);
            //robot.SlideR.setPosition(0.4);
        }else if(gamepad2.dpad_left){
            //robot.SlideL.setPosition(slidel_mid);
            //robot.SlideR.setPosition(slider_mid);
        }else if (gamepad2.dpad_down){
            //robot.SlideL.setPosition(0);
            //robot.SlideR.setPosition(0);
        }


        // duck
        if (gamepad2.x){
            //robot.Duck.setPower(-0.4);
        }else{
            //robot.Duck.setPower(0);
        }


        // Intake
        if (gamepad2.a){ // eat
            //robot.Intake.setPower(-1);
        }else if (gamepad2.b){ // not eat
            //robot.Intake.setPower(.5);
        }else{
            //robot.Intake.setPower(0);
        }


        // Slide
        if (gamepad2.left_bumper){
            //robot.Slide.setPower(1.5);
        }else if (gamepad2.right_bumper){
            //robot.Slide.setPower(-0.7);
        }else{
            //robot.Slide.setPower(0);
        }

        // Cap
        if (gamepad1.y){
            //robot.Cap.setPosition(1);
        }else if (gamepad1.b){
            //robot.Cap.setPosition(0.6);
        }else if (gamepad1.a){
            //robot.Cap.setPosition(0.35);
        }else if (gamepad1.x){
            //robot.Cap.setPosition(0);
        }

        // if (gamepad1.x) {
        //     robot.Rback.setPower(1);
        // }
        // else if(gamepad1.y){
        //     robot.Rback.setPower(-1);
        // }else{
        //     robot.Rback.setPower(0);
        // }
        if(gamepad2.y){
            // biggo(2000, 1);
            // sleep(3000);
            // robot.SlideR.setPosition(0.4);
            // sleep(2000);
            // robot.SlideR.setPosition(0.125);
            // sleep(2000);
            // biggo(750, -0.4);
            // telemetry.addLine("biggo");
            // sleep(2000);
            // telemetry.addLine("done");
        }
    }


    @Override
    public void stop () {

    }
}
