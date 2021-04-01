package frc.robot.subsystems;

import java.util.List;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.pathrecognizer.VisionFieldLayoutRecognizer;
import frc.pathrecognizer.VisionPathName;
//import sun.awt.www.content.image.png;
import frc.pathrecognizer.AutonomousPath;
import frc.pathrecognizer.PixyCameraConnector;
import frc.pathrecognizer.FieldVisionInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathRecognizerSubsystem extends SubsystemBase {

    private VisionFieldLayoutRecognizer visionFieldLayoutRecognizer = new VisionFieldLayoutRecognizer();    
    private PixyCameraConnector pixyCameraConnector;
    private List<FieldVisionInput> fieldVisionInputList; 

    public PathRecognizerSubsystem(){
        initialize();
    }

    public void initialize() {  
        pixyCameraConnector = new PixyCameraConnector();                       
    }

    public AutonomousPath getAutonomousPath(){
        AutonomousPath autonomousPath = new AutonomousPath(VisionPathName.CouldNotDeterminePath);

        if (pixyCameraConnector.getPixyConnectionStatus()){
            fieldVisionInputList = pixyCameraConnector.getFieldVisionInput();
            autonomousPath = visionFieldLayoutRecognizer.detectPose(fieldVisionInputList);
        }
        
        SmartDashboard.putString("Pixy Detected Path ", autonomousPath.getPathName().toString());
        SmartDashboard.putBoolean("Pixy Camera Connection Status", pixyCameraConnector.getPixyConnectionStatus());
     
        return autonomousPath;         
    }
}
