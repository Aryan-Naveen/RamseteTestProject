package frc.pathrecognizer;

import java.util.List;
import java.util.ArrayList;
import frc.robot.logger.DataLogger;
import frc.robot.logger.DataLoggerFactory;


 public class VisionFieldLayoutRecognizer
    implements PixyFieldPoseResolverInterface{

    private DataLogger logger = DataLoggerFactory.getLoggerFactory().createDataLogger(VisionFieldLayoutRecognizer.class.getName()); 
    //Cannot instantiate the type List if I say new List<DistanceRange>, I get this error
    private List<DistanceRange> distanceRanges;
    public static final double FOCAL_LENGTH = 257.14;
    public static final double WIDTH = 177.8; //millimeters
     
    
    public VisionFieldLayoutRecognizer() {
      setupInitialDistanceRanges();
    }
    public void setupInitialDistanceRanges(){  
      this.distanceRanges = List.of(new DistanceRange(2590.8, 3048, VisionPathName.PathRedA),
      new DistanceRange(4907.28, 5168.24, VisionPathName.PathRedB),
      new DistanceRange(4100, 4400.92, VisionPathName.PathBlueA),
      new DistanceRange(5334, 5725.55, VisionPathName.PathBlueB)
    
      );

  }
  private double computeAverageDistance(List<FieldVisionInput> fieldVisionInputParameter){
    
    double DISTANCE = 0;
    List<Double> pixels = new ArrayList<>();
    
          for(FieldVisionInput fieldVisionInput: fieldVisionInputParameter){

          pixels.add(fieldVisionInput.getObjectWidth());
                            
        }
      
        for (Double pixarea: pixels){
        if (pixarea > 0){
         DISTANCE = DISTANCE + (WIDTH*FOCAL_LENGTH)/pixarea;
        }
      }

      if (pixels.size() > 0){
        DISTANCE = DISTANCE/pixels.size();
      }
      return DISTANCE;
  }

  public AutonomousPath detectPose(List<FieldVisionInput> fieldVisionInputParameter) {
        
      AutonomousPath autonomousPath;
      VisionPathName visionPathName = VisionPathName.CouldNotDeterminePath;
       
      // Just for reference the formula to calculate the focal length
      //dblFocalLength = (dblPixels*dblDistance)/dblWidth;   
      
      double averageDistance = computeAverageDistance(fieldVisionInputParameter);
      logger.log("Number of Blocks:", fieldVisionInputParameter.size());
      logger.log("Average Distance:", averageDistance);

        for(DistanceRange dr:distanceRanges){

          if (dr.isInRange(averageDistance)){
            visionPathName = dr.getPath();
            break;
          }
          
        }
        
      autonomousPath = new AutonomousPath(visionPathName);

      return autonomousPath;

    }

 }