package frc.pathrecognizer;

import java.util.List;

public interface PixyFieldPoseResolverInterface{

    public AutonomousPath detectPose(List<FieldVisionInput> fieldVisionInput);
    
}