package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SleeveDetectionPipeline extends OpenCvPipeline {
    int magHue = 140;
    int cyaHue = 25;
    int yelHue = 90; // Only used to display box
    int range = 10;
    Color color;
    double TOP = 0.5;
    double BOTTOM = 0.8;
    double LEFT = 0.5;
    double RIGHT = 1;

    public enum Color {
        YELLOW,
        CYAN,
        MAGENTA,
    }

    int sumHues(Mat input, Scalar minScale, Scalar maxScale){

        Mat output = new Mat(input.rows(), input.cols(), input.type());
        Core.inRange(input, minScale, maxScale, output);
        return (int)Core.sumElems(output).val[0]/255;
    }

    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HSV);

        Mat cropped = input.submat((int)(240*TOP), (int)(240*BOTTOM), (int)(320*LEFT), (int)(320*RIGHT));

        int magCount = sumHues(cropped, new Scalar(magHue - range / 2, 30, 0), new Scalar(magHue + range / 2, 140, 255));
        int cyaCount = sumHues(cropped, new Scalar(cyaHue - range / 2, 60, 0), new Scalar(cyaHue + range / 2, 140, 255));
        int yelCount = 500;

        Scalar rectCol;
        if(magCount > cyaCount && magCount > yelCount){
            // Mag
            color = Color.MAGENTA;
            rectCol = new Scalar(magHue, 255, 255);
        }else if(cyaCount > yelCount){
            // Cya
            color = Color.CYAN;
            rectCol = new Scalar(cyaHue, 255, 255);
        }else{
            // Yel
            color = Color.YELLOW;
            rectCol = new Scalar(yelHue, 255, 255);
        }


        Imgproc.rectangle(
                input,
                new Point(320*LEFT, 240*BOTTOM),
                new Point(320*RIGHT, 240*TOP),
                rectCol, 4
        );
        Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2BGR);
        return input;
    }



}
