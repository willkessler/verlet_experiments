// this snippet written after studying this basic article: //<>//
// http://datagenetics.com/blog/july22018/index.html

// todo: armDampener should increase when angle gets small

int BLOB_R = 5;
float gravity = 0.01;
PVector [] points;
PVector [] prevPoints;
float shoulderLength = 100;
float armLength = 145;
int NS = 1;
float angle = 0;
float [] maxStickAngles = { 45, 50 };
PVector windowCenter;
float initialAngle = 2;
float restingAngle = -10;
float initialAngleVel = -4;
float angleVel;
float tau = .15;
float angleVelDampener = 1;
float mass = 30;

float maxShoulderAngle = -100;
boolean didConstrain = false;


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
  return result;
}


void setup() {
  size(800, 650, P2D);
  windowCenter = new PVector(width/2, height/2);
  points = new PVector[2];
  points[0] = new PVector(shoulderLength, 0);
  points[1] = new PVector(shoulderLength + armLength, 0);
  prevPoints = new PVector[2];
  prevPoints[1] = new PVector(shoulderLength + armLength, 0);
  prevPoints[1].rotate(radians(initialAngle));
  angleVel = initialAngleVel;
}

void updateShoulder() {
 // for the shoulder, apply spring force as tangential to the circle at the shoulder length, where
  // accelVector = force / mass, and force = -1 * tau * spring_angle , tau = torsional spring constant
  
  PVector restingVector = new PVector(cos(radians(restingAngle)), sin(radians(restingAngle)));
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

void updateArm() {
  // move end of "elbow"
 // float dampener = computeDampener();
  float dampener = 1;
  float vx = (points[1].x - prevPoints[1].x) * dampener;
  float vy = (points[1].y - prevPoints[1].y) * dampener + gravity;
  prevPoints[1].set(points[1].x, points[1].y);
  points[1].set(points[1].x + vx, points[1].y + vy);
}

void updatePoints() {
  updateShoulder();
  updateArm();
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
  didConstrain = false;
  //float prevArmLength =  dist(points[0].x, points[0].y, points[1].x, points[1].y); 
  //PVector prevPrev = new PVector(prevPoints[1].x, prevPoints[1].y);
  //PVector prevPoint = new PVector(points[1].x, points[1].y);
  PVector p1, p2;
  p1 = new PVector(points[0].x, points[0].y);
  p2 = new PVector(points[1].x, points[1].y);
  p2.sub(p1);

  PVector p3 = new PVector(prevPoints[1].x, prevPoints[1].y);
  PVector p4 = new PVector(points[1].x, points[1].y);
  p3.sub(points[0]);
  p4.sub(points[0]);
  FloatDict angleBetweenPrevAndNext = angleBetweenVectors(p3, p4);
  float prevNextAngleSign = angleBetweenPrevAndNext.get("sign");
  FloatDict angleBetweenShoulderAndArm = angleBetweenVectors(p1, p2);
  boolean checkUpper = (angleBetweenShoulderAndArm.get("sign") < 0);
  int isAboveAlignment = angleBetweenShoulderAndArm.get("sign") < 0 ? 0 : 1;
  float maxStickAngle = maxStickAngles[isAboveAlignment];
  //println("angleSign:", angleBetweenShoulderAndArm.get("sign"), "angleBetweenShoulderAndArm:", angleBetweenShoulderAndArm.get("angle"));

  // we only want to constrain if the direction of motion was *towards* the maximum in question
  // if it's *away* from the max line then we want to let it keep going.
  // to determine which max line we're interested in, compute the angle between the shoulder and
  // the arm. if the sign is positive, it's the upper max line, otherwise it's the lower max line

  println("checkUpper:", checkUpper, "prevNextAngleSign:", prevNextAngleSign);
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
    
  }
}

void render() {

  fill(255);
  //float yDiff = points[1].y - prevPoints[1].y;
  text(prevPoints[1].y + " : " + points[1].y + (didConstrain ? " : C" : ""), 100,100);
  stroke(255, 0, 0);

  translate(windowCenter.x - width/4, windowCenter.y);

  //draw shoulder and arm segments
  line(0, 0, points[0].x, points[0].y );
  line(points[0].x, points[0].y, points[1].x, points[1].y);


  //draw ellipses at joints
  stroke(164, 164, 164); 
  fill(255, 255, 0);
  ellipse(0, 0, BLOB_R*2, BLOB_R*2);
  for (int i=0; i<2; i++) {
    ellipse(points[i].x, points[i].y, BLOB_R*2, BLOB_R*2);
  }

  translate(-windowCenter.x, -windowCenter.y);
  
  
}

void draw() {

  background(0, 0, 0, 0);
  noFill();
  stroke(255);

  updatePoints();
  updateSticks();
  constrainAngles();
  render();
  //delay(250);
}
