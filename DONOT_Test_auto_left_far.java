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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Test_auto_left_far", group="FTC 2023")

public class DONOT_Test_auto_left_far extends Nav {

    Test_init robot = new Test_init();

    public void initial(){

        robot.init(hardwareMap);
        robot.Lfront.setPower(0);
        robot.Rfront.setPower(0);
        robot.Rback.setPower(0);
        robot.Lback.setPower(0);
        robot.SSlide.setPower(0);
        robot.Intake.setPosition(0.97);
        //robot.Intake.setPower(0);
        //robot.Slide.setPower(0);
        
        robot.Slide.setTargetPosition(0);
        //robot.SlideR.setPosition(0.128);
        //robot.Cap.setPosition(0);
        
        robot.runModeSet("reset");
        robot.runModeSet("encoder");
        
        robot.Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        robot.SSlide.setTargetPosition(0);
        robot.SSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.SSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.Slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.Slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.Slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public void biggo (int position, double power) {
        robot.Slide.setTargetPosition(position);
        robot.Slide.setPower(power);
        //robot.Slide2.setTargetPosition(position);
        //robot.Slide2.setPower(power);        
    }
    public void smallgo (int position, double power) {
        robot.SSlide.setTargetPosition(position);
        robot.SSlide.setPower(power);
        //robot.Slide2.setTargetPosition(position);
        //robot.Slide2.setPower(power);        
    }
    

//public void go_forward(double inches_to_travel, double heading, double speed, boolean gotocrater) {
//public double go_sideways(double angledegrees, double heading, double power, double inches) {
//public void turn_to_heading(double target_heading) {
    
    @Override
    public void runOpMode() {
        final double PILLAR_RIGHT=0;
        final double PILLAR_FRONT=0;
        int tallest=11500;
        initial();
        Nav_Init();
        waitForStart();
        //smallgo(-3000,0.3);
        //sleep(100000);
        //turn_to_heading(90);
        robot.Intake.setPosition(0.03);
        sleep(500);
        biggo(tallest,1);
        
        //sleep(10000000);
        //sleep(2000);
        go_sideways(270,0,0.5,13);
        sleep(1000);
        int red=(robot._cs.red());
        int blue =(robot._cs.blue());
        int green=(robot._cs.green());
        int pos=0;
        pos=0;
        if(Math.max(red,Math.max(green,blue))==blue){
            pos=3;
        }else if(Math.max(red,Math.max(green,blue))==green){
            pos=2;
        }else{
            pos=1;
        }
        smallgo(-3000,1);
        go_sideways(270,0,0.5,52);
        go_sideways(90,0,0.5,10);
        turn_to_heading(270);
        //biggo(7200,1);
        go_sideways(90,270,0.5,12+PILLAR_RIGHT);//first pillar go left
        //sleep(3000);
        go_forward(7.3+PILLAR_FRONT,270,0.5,false);//CONSTANT ALERT
        go_forward(1,270,0.2,false);//yes
        sleep(1000);
        smallgo(-2000,1);
        //sleep(500);
        robot.Intake.setPosition(0.97);
        //sleep(500);
        smallgo(-3000,1);
        sleep(500);
        go_sideways(180,270,0.5,6+PILLAR_FRONT);
        //smallgo(-3000,1);
        
        //sleep(2000);
        /*
            FIRST DONE
        */
        
        biggo(3000,0.8);
        smallgo(-380,1);
        //smallgo(0,0.3);
        sleep(3000);
        go_sideways(270,270,0.5,10+PILLAR_RIGHT);
        //sleep(2000);
        //sleep(10000);
        turn_to_heading(180);
        
        //sleep(100000);
        go_sideways(270,180,0.5,1);
        go_forward(25,180,0.4,false);
        smallgo(0,1);
        biggo(2500,0.5);
        go_forward(1,180,0.2,false);
        sleep(1000);
        //sleep(100000000);
        robot.Intake.setPosition(0.03); 
        //sleep(100000000);
        sleep(500);
        smallgo(-3000,1);
        biggo(tallest,1);
        sleep(1000);
        go_sideways(180,180,1,20);
        turn_to_heading(270);
        go_sideways(90,270,1,10+PILLAR_RIGHT);
        
        go_forward(8,270,1,false);
        go_forward(3.3,270,1,false);//CONSTANT ALERT
        sleep(1000);
        smallgo(-2000,1);
        sleep(500);
        robot.Intake.setPosition(0.97);
        sleep(500);
        smallgo(-3000,1);
        sleep(500);
        go_sideways(180,270,0.5,6);
        
        sleep(100000000);
        if(pos==1) go_sideways(270,90,1,10);
        if(pos==2) go_sideways(90,90,1,10);
        if(pos==3) go_sideways(90,90,1,35);
        go_sideways(180,90,1,10);
        sleep(100000000);
    }

}
