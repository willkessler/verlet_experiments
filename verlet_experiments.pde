// this snippet written after studying this basic article: //<>//
// http://datagenetics.com/blog/july22018/index.html

// todo: armDampener should increase when angle gets small DONE
// todo: add muscular "boost" to arm at faster shoulder speeds so it keeps up
// todo: make top angle limit for arm less than bottom limit
// todo: reapply dampener to arm

import org.gicentre.utils.stat.*;    // For chart classes.
float [] graphX;
float [] graphY;
 
// Displays a simple line chart representing a time series.
XYChart lineChart;

int BLOB_R = 5;
float gravity = 0.01;
PVector [] points;
PVector [] prevPoints;
float shoulderLength = 150;
float armLength = 195;
int NS = 1;
float angle = 0;
float [] maxStickAngles = { 20, 85 };
PVector windowCenter;
float initialAngle = 2;
float shoulderRestingAngle = -10;
float armRestingAngle = 30;
float initialAngleVel = -4;
float angleVel;
float tau = .15;
float angleVelDampener = 1;
float mass = 30;
int stepsSinceConstraining = 0;
boolean stepsIncreasing = false;

boolean didConstrain = false;
PVector muscleBoostVector;

FloatDict angleBetweenShoulderAndArm, angleBetweenPrevAndNext;
float armAngleToRestingAngle, gain;
boolean armGoingUp = false;
String conditions;

// https://forum.processing.org/two/discussion/3811/what-is-the-alternative-in-processing
int sign(float f) {
  if (f==0) return(0);
  return(int(f/abs(f)));
}

// see: https://www.euclideanspace.com/maths/algebra/vectors/angleBetween/
FloatDict angleBetweenVectors(PVector v1, PVector v2) {        
  FloatDict result = new FloatDict();
  result.set("angle", 0);
  result.set("sign", 1);
  result.set("signedAngle", 0);
  PVector v1Norm = new PVector(v1.x, v1.y);
  PVector v2Norm = new PVector(v2.x, v2.y);
  v1Norm.normalize();
  v2Norm.normalize();
  
  PVector zeroCheck = PVector.sub(v1Norm, v2Norm);
  if (zeroCheck.mag() < 0.01) {
    return result;
  }
  float dp = v1Norm.dot(v2Norm);
  result.set("angle", degrees(acos(dp)));
  // cross prod of 2 2d vecs, cf source of https://chipmunk-physics.net/
  // also see https://stackoverflow.com/questions/243945/calculating-a-2d-vectors-cross-product#:~:text=You%20can't%20do%20a,vectors%20on%20the%20xy%2Dplane.
  result.set("sign", sign(v1.x * v2.y - v1.y * v2.x));
  result.set("signedAngle", result.get("angle") * result.get("sign"));
  return result;
}


void setup() {
  size(800, 1000, P2D);
  windowCenter = new PVector(width/2, height/2);
  points = new PVector[2];
  points[0] = new PVector(shoulderLength, 0);
  points[1] = new PVector(shoulderLength + armLength, 0);
  prevPoints = new PVector[2];
  prevPoints[1] = new PVector(shoulderLength + armLength, 0);
  prevPoints[1].rotate(radians(initialAngle));
  angleVel = initialAngleVel;
  muscleBoostVector = new PVector(0,0);

  // Both x and y data set here.  
  int graphRange = 1000;
  graphX = new float[graphRange];
  graphY = new float[graphRange];
  for (int i = 0; i < graphRange; ++i) {
    graphX[i] = i;
    graphY[i] = 0;
  }
  lineChart = new XYChart(this);
  lineChart.setData(graphX, graphY);
   
  // Axis formatting and labels.
  lineChart.showXAxis(true); 
  lineChart.showYAxis(true); 
  lineChart.setMinY(0);
     
  // Symbol colours
  lineChart.setPointColour(color(180,50,50,100));
  lineChart.setPointSize(1);
  lineChart.setLineWidth(1);
  
}

// compute the boost based on the steps from the didConstrain moment.
// width of the gaussian fn (c) should be a function of the shoulder angle.
float computeMuscleBoostGaussian(int steps) {
  if (steps == 0) {
    return 1.0;
  }
  float[] a_vals = { 0.15, 0.05 };
  float a = a_vals[ (armGoingUp ? 1 : 0) ];
  float d = 5;
  float c = 1.5;
  float expArg = ( -1 * ((steps - d) * (steps - d)) ) / (2 * c * c);
  float g = a * exp(expArg) + 1;
  return g;
}

// compute a muscle boost based on the sin of the angle of the shoulder, as a proportion to 
// the tangent of the arm vector in the direction of the shoulder's motion. 

// UPDATE: hit the "power" when the arm reaches parallel with the shoulder. "lock" the arm
// when the shoulder reaches a certain minimum angle going down and similarly going up

void computeMuscleBoostAngleSin() {
  muscleBoostVector.set(0,0);
  armAngleToRestingAngle = angleBetweenShoulderAndArm.get("signedAngle") - armRestingAngle;
  boolean mustBoost;
  boolean cond1, cond2, cond3, cond4, cond5, cond6;
  cond1 = (armAngleToRestingAngle < 0) && (angle < 0) && (angleVel < 0); // arm bent up slightly too high before shoulder comes back down
  cond2 = (armAngleToRestingAngle < 0) && (angle < 0) && (angleVel > 0); // arm bent up too high as shoulder begins to come down
  cond3 = (armAngleToRestingAngle > 40);                 // arm bent down way too far generally
  cond4 = (armAngleToRestingAngle > 20) && (armAngleToRestingAngle < 40) && (angle < 0) && (angleVel > 0); // arm going down, above horizon, arm bent too far down
  cond5 = (angleVel < 0) && (armAngleToRestingAngle > 20); // shoulder going up, and arm bent too far down... 
  cond6 = (angle > 0) && (angleVel < 0);                  // shoulder below horizon, and shoulder going up or near to going up

  mustBoost = cond1 || cond2 || cond3 || cond4;
  conditions = (cond1 ? "C1 " : " ") + (cond2 ? "C2 " : " ") + (cond3 ? "C3 " : " ") + (cond4 ? "C4 " : "") + (cond5 ? "C5 " : "") + (cond6 ? "C6 " : "");
  
  if (mustBoost) {
    float angleSin = sin(radians(angle));
    //gain = (((angleVel > 0) && (angle < 0)) ? 6 : 2); // gain high if shoulder going down above horizon only
    gain = 
      (cond1 ? 1 : 0) * 3 +
      (cond2 ? 1 : 0) * 4 +
      (cond3 ? 1 : 0) * 4 +
      (cond4 ? 1 : 0) * 3 ;
    //      (cond5 ? 1 : 0) * 2 +
    //      (cond6 ? 1 : 0) * 1;

    PVector p1;
    p1 = new PVector(points[0].x, points[0].y);
    muscleBoostVector.set(points[1].x, points[1].y);
    muscleBoostVector.sub(p1);
    muscleBoostVector.normalize();

    float tangentRot = -90;
    muscleBoostVector.rotate(radians(tangentRot));
    //float armAngleGainMultiplier = gain * pow(radians(angleBetweenShoulderAndArm.get("signedAngle")), 1.5);
    float armAngleGainMultiplier = gain * sin(radians(armAngleToRestingAngle));
    muscleBoostVector.mult(armAngleGainMultiplier);
  }
}

// Compute angle between shoulder and arm, and
// between the prev and next point of the arm to the end of the shoulder.
void computeArmSegmentAngles() {
  PVector p1, p2;
  p1 = new PVector(points[0].x, points[0].y);
  p2 = new PVector(points[1].x, points[1].y);
  p2.sub(p1);

  angleBetweenShoulderAndArm = angleBetweenVectors(p1, p2);

  PVector p3 = new PVector(prevPoints[1].x, prevPoints[1].y);
  PVector p4 = new PVector(points[1].x, points[1].y);
  p3.sub(points[0]);
  p4.sub(points[0]);
  angleBetweenPrevAndNext = angleBetweenVectors(p3, p4);

  armGoingUp = (angleBetweenPrevAndNext.get("sign") < 0);

  
}

// at faster shoulder speeds, arm doesn't "have time" to catch up and swing throughout its motion,
// which creates unrealistic motion of the arm. the arm needs a muscular "boost" when shoulder motions
// are quicker, in order to keep up.
  
void updateShoulder() {
 // for the shoulder, apply spring force as tangential to the circle at the shoulder length, where
  // accelVector = force / mass, and force = -1 * tau * spring_angle , tau = torsional spring constant
  
  PVector restingVector = new PVector(cos(radians(shoulderRestingAngle)), sin(radians(shoulderRestingAngle)));
  PVector shoulderVector = new PVector(points[0].x, points[0].y);
  FloatDict springAngle = angleBetweenVectors(restingVector, shoulderVector);
  float tauForce = -1 * tau * springAngle.get("angle") * springAngle.get("sign");
  angleVel += tauForce / mass;
  angleVel *= angleVelDampener;
  angle += angleVel;
  
  // set shoulder position
  float radAngle = radians(angle);
  points[0].set(cos(radAngle) * shoulderLength, sin(radAngle) * shoulderLength);
  
}

// compute dampener as the product of the angleFactors,
// where each angleFactor is the 1 - 1.0/angle between p0->prevPoint and p0->maxAngle

float computeDampener() {

  PVector p1 = new PVector(prevPoints[1].x, prevPoints[1].y);
  p1.sub(points[0]);
  PVector p2 = new PVector(points[0].x, points[0].y);
  p2.normalize();
  PVector p3 = new PVector(p2.x,p2.y);
  p3.rotate(-1 * maxStickAngles[0]);
  FloatDict angleToUpperBoundary = angleBetweenVectors(p1,p3);
  PVector p4 = new PVector(p2.x,p2.y);
  p4.rotate(maxStickAngles[1]);
  FloatDict angleToLowerBoundary = angleBetweenVectors(p1,p4);

  // 1 - 1 / (.25 * angle + 1)
  float spread = maxStickAngles[0] + maxStickAngles[1];
  float upperAngle = angleToUpperBoundary.get("angle");
  float lowerAngle = angleToLowerBoundary.get("angle");
  float curveFlattener = .4;
  float dampener1 = 1.0 - 1.0 / (curveFlattener * upperAngle + 1);
  float dampener2 = 1.0 - 1.0 / (curveFlattener * lowerAngle + 1);
  
  // if headed upwards, use upper dampener, otherwise use lower
  float yDir = points[1].y - prevPoints[1].y;
  float finalDampener = (yDir > 0 ? dampener1 : dampener2);
  //float finalDampener = (upperAngle < lowerAngle ? dampener1 : dampener2); 
  
  println("YDir", yDir, "Dampeners:", "[", upperAngle, dampener1, "] [", lowerAngle, dampener2, "] : ", finalDampener);

  return finalDampener;
}

// Move end of "elbow"
void updateArm(boolean applyBoost) {
  PVector bv = new PVector(0,0);
  if (applyBoost) {
    bv.set(muscleBoostVector.x, muscleBoostVector.y);
  }
  float vx = (points[1].x - prevPoints[1].x) + bv.x;
  float vy = (points[1].y - prevPoints[1].y) + gravity + bv.y;
  
  prevPoints[1].set(points[1].x, points[1].y);
  points[1].set(points[1].x + vx, points[1].y + vy);
}

void updatePoints() {
  updateShoulder();
  updateArm(true);
}


void updateSticks() {
  float dx, dy, dist, diff, percent, offx, offy;
  dx = points[1].x - points[0].x;
  dy = points[1].y - points[0].y;
  dist = sqrt((dx*dx)+(dy*dy));
  diff = armLength - dist;
  // percent = diff/dist/2; // divide by two only if moving both ends of a stick
  percent = diff / dist;
  offx = dx*percent;
  offy = dy*percent;

  points[1].x += offx;
  points[1].y += offy;
  //println("updateSticks:", points[1].x, points[1].y);
}


// constrain angle between shoulder and elbow
void constrainAngles() {
  if (stepsIncreasing) {
    stepsSinceConstraining++;
  }


  didConstrain = false;

  float prevNextAngleSign = angleBetweenPrevAndNext.get("sign");

  boolean checkUpper = (angleBetweenShoulderAndArm.get("sign") < 0);
  int isAboveAlignment = angleBetweenShoulderAndArm.get("sign") < 0 ? 0 : 1;
  float maxStickAngle = maxStickAngles[isAboveAlignment];
  //println("angleSign:", angleBetweenShoulderAndArm.get("sign"), "angleBetweenShoulderAndArm:", angleBetweenShoulderAndArm.get("angle"));

  // we only want to constrain if the direction of motion was *towards* the maximum in question
  // if it's *away* from the max line then we want to let it keep going.
  // to determine which max line we're interested in, compute the angle between the shoulder and
  // the arm. if the sign is positive, it's the upper max line, otherwise it's the lower max line

  //println("checkUpper:", checkUpper, "prevNextAngleSign:", prevNextAngleSign);
  if ( (angleBetweenShoulderAndArm.get("angle") <= maxStickAngle) ||
       (prevNextAngleSign > 0 && checkUpper) ||
       (prevNextAngleSign < 0 && !checkUpper) )  {
    // dampen by a function of the angle diff
    float dampener = 1.0 - (1.0 / angleBetweenShoulderAndArm.get("angle"));
    float f1 = angleBetweenShoulderAndArm.get("angle") * dampener;
} else { 
    // exceeded how far we can rotate, so reverse direction.
    // Calculate angle diff between prevPoints[1] and points[1]. 
    // place prevPoints[1] at maxStickAngle
    // rotate same amount backwards to place points[1]
    float f1 = (maxStickAngle - angleBetweenPrevAndNext.get("angle")) * angleBetweenShoulderAndArm.get("sign");
    float f1r = radians(f1);

    PVector newP1 = new PVector(armLength,0);
    newP1.rotate(radians(angle)); // rotate to the shoulder angle
    newP1.rotate(f1r);   // additionally, rotate to the maxStickAngle minus the overshoot
    newP1.add(points[0]);
    points[1].set(newP1.x, newP1.y);
    
    // place previous point at maxStickAngle position by taking vector to end of shoulder, 
    // rotating to the arm's rotation, then translating to the end of the shoulder
    float f2 = radians(maxStickAngle * angleBetweenShoulderAndArm.get("sign"));
    PVector newPrev = new PVector(points[0].x, points[0].y);
    newPrev.rotate(f2);
    newPrev.add(points[0]);
    prevPoints[1].set(newPrev.x, newPrev.y);
 
    //println("Reset, f1:", f1, "angleBetween:", angleBetweenPrevAndNext.get("angle"),
    //        "prevArmLength:", prevArmLength,
    //        "armLength:", dist(points[0].x, points[0].y, points[1].x, points[1].y));
    //println("prevPrev:", prevPrev, "prevPoint:", prevPoint);
    //println("newPrev:", prevPoints[1], "newPoint:", points[1]);
    didConstrain = true;
    stepsIncreasing = true;
    stepsSinceConstraining = 0;
    
  }

}

void render() {

  fill(255);
  //float yDiff = points[1].y - prevPoints[1].y;
  //text(prevPoints[1].y + " : " + points[1].y + (didConstrain ? " : C" : ""), 50,100);
  //text("boost:" + computeMuscleBoost(stepsSinceConstraining), 50,80);
  float radAngle = radians(angle);
  int fSize = 24;
  int fSizeBuffered = fSize + 4;
  int leftMargin = 25;
  textSize(fSize);
  text("Shoulder Angle:" + round(angle),                                                leftMargin, height - 6 * fSizeBuffered);
  text("Anglevel:" + angleVel,                                                          leftMargin, height - 5 * fSizeBuffered);
  text("Shoulder-arm Angle:" + round(angleBetweenShoulderAndArm.get("signedAngle")),    leftMargin, height - 4 * fSizeBuffered);
  text("Shoulder-Arm Angle Distance to Resting Angle:" + round(armAngleToRestingAngle), leftMargin, height - 3 * fSizeBuffered);
  text("Gain:" + round(gain) + " Conditions:" + conditions,                              leftMargin, height - 2 * fSizeBuffered);
  stroke(0,255,0);
  strokeWeight(round(6 * gain / 4));

  translate(windowCenter.x - width/4, windowCenter.y - 20);

  //draw shoulder and arm segments
  line(0, 0, points[0].x, points[0].y );
  line(points[0].x, points[0].y, points[1].x, points[1].y);

  PVector muscleBoostVectorRendered = new PVector(muscleBoostVector.x,  muscleBoostVector.y);
  muscleBoostVectorRendered.mult(100);
  muscleBoostVectorRendered.add(points[1]);

  stroke(55,55,0);
  line(points[1].x, points[1].y, muscleBoostVectorRendered.x, muscleBoostVectorRendered.y);

  //draw ellipses at joints
  stroke(164, 164, 164); 
  fill(255, 255, 0);
  ellipse(0, 0, BLOB_R*2, BLOB_R*2);
  for (int i=0; i<2; i++) {
    ellipse(points[i].x, points[i].y, BLOB_R*2, BLOB_R*2);
  }

  translate(-windowCenter.x, -windowCenter.y);
  
  textSize(9);
  //lineChart.draw(15,width/2, width/2 -10,height/2);
   
}

void draw() {

  background(0, 0, 0, 0);
  noFill();
  stroke(255);

  computeArmSegmentAngles();
  computeMuscleBoostAngleSin();
  updatePoints();
  updateSticks();
  //constrainAngles();
  render();
  delay(30);
}
