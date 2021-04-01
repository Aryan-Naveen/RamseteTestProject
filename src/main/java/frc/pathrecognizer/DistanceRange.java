package frc.pathrecognizer;

public class DistanceRange{

  private double lowerbound = 0;
  private double upperbound = 0;
  private VisionPathName path;
  
  public DistanceRange ( double lowerbound, double upperbound, VisionPathName path ){
    
    this.lowerbound = lowerbound;
    this.upperbound= upperbound;
    this.path = path;  
  
  }
  
  public boolean isInRange(double value ){

    if (( value >= lowerbound ) && (value <= upperbound )){
      return true;
    }
    else{
      return false;
    }

  }

public VisionPathName getPath() {
	  return path;
}

public void setPath(VisionPathName path) {
	  this.path = path;
}

    
}