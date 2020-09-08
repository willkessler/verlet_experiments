// this snippet written after studying this basic article: //<>//
// http://datagenetics.com/blog/july22018/index.html

// todo: dampener should increase when angle gets small

int BLOB_R = 5;
int cctr = 0;
PVector [] points;
PVector [] prevPoints;
float circleRad = 100;
float stickLength = 100;
int NS = 1;
float angle = 0;
float angleInc = 4;
float angleRange = 65;
float maxStickAngle = 30;
PVector windowCenter;
float dampener = .99;
float gravity = 0.5;
float initialAngle = 2;


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
  PVector zeroCheck = PVector.sub(v1, v2);
  if (zeroCheck.mag() < 0.01) {
    return result;
  }
  float dp = v1.dot(v2);
  float denom = v1.mag() * v2.mag();
  result.set("angle", degrees(acos(dp/denom)));
  result.set("sign", sign(v1.x * v2.y - v1.y * v2.x));
  return result;
}

void setup() {
  size(800, 650, P2D);
  windowCenter = new PVector(width/2, height/2);
  points = new PVector[2];
  points[0] = new PVector(circleRad, 0);
  points[1] = new PVector(circleRad + stickLength, 0);
  prevPoints = new PVector[2];
  prevPoints[1] = new PVector(circleRad + stickLength, 0);
  prevPoints[1].rotate(radians(initialAngle));
}


void updatePoints() {

  angle += angleInc;
  if (angle > angleRange) {
    angle = angleRange;
    angleInc *= -1;
  } else if (angle < -angleRange) {
    angle = -angleRange;
    angleInc *= -1;
  }
  //if ((angle > 720) || (angle < 0)) {
  //  angleInc *= -1;
  //}

  // move "shoulder" by apply spring force
  float radAngle = radians(angle);
  points[0].set(cos(radAngle) * circleRad, sin(radAngle) * circleRad);

  // move end of "elbow"
  float vx = (points[1].x - prevPoints[1].x) * dampener;
  float vy = (points[1].y - prevPoints[1].y) * dampener + gravity;
  prevPoints[1].set(points[1].x, points[1].y);
  points[1].set(points[1].x + vx, points[1].y + vy);
}


void updateSticks() {
  float dx, dy, dist, diff, percent, offx, offy;
  dx = points[1].x - points[0].x;
  dy = points[1].y - points[0].y;
  dist = sqrt((dx*dx)+(dy*dy));
  diff = stickLength - dist;
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
  float prevDist =  dist(prevPoints[1].x, prevPoints[1].y, points[1].x, points[1].y); 
  PVector prevPrev = new PVector(prevPoints[1].x, prevPoints[1].y);
  PVector prevPoint = new PVector(points[1].x, points[1].y);
  PVector p1, p2;
  p1 = new PVector(points[0].x, points[0].y);
  p2 = new PVector(points[1].x, points[1].y);
  p2.sub(p1);
  //p1.normalize();
  //p2.normalize();

  //PVector p5 = new PVector(prevPoints[1].x, prevPoints[1].y);
  //PVector p6 = new PVector(points[1].x, points[1].y);
  //float angleBetweenP5andP6 = angleBetweenVectors(p5, p6);
  //println("Loop, angleBetween:", angleBetweenP5andP6);


  // cross prod of 2 2d vecs, cf source of https://chipmunk-physics.net/
  // also see https://stackoverflow.com/questions/243945/calculating-a-2d-vectors-cross-product#:~:text=You%20can't%20do%20a,vectors%20on%20the%20xy%2Dplane.
  //float angleSign = sign(p1.x * p2.y - p1.y * p2.x);
  FloatDict angleBetweenCrankAndStick = angleBetweenVectors(p1, p2);
  //println("angleSign:", angleBetweenCrankAndStick.get("sign"), "angleBetweenCrankAndStick:", angleBetweenCrankAndStick.get("angle"));
  if (angleBetweenCrankAndStick.get("angle") > maxStickAngle) {
    cctr++;
    //println("cctr:", cctr);
    if (cctr == 4) {
      println("reverse bottom");
    }

    // calculate angle diff between prevPoints[1] and points[1]. 
    // place prevPoints[1] at maxStickAngle
    // rotate same amount backwards to place points[1]
    PVector p3 = new PVector(prevPoints[1].x, prevPoints[1].y);
    PVector p4 = new PVector(points[1].x, points[1].y);
    p3.sub(points[0]);
    p4.sub(points[0]);
    FloatDict angleBetweenPrevAndNext = angleBetweenVectors(p3, p4);
    float f1 = (maxStickAngle - angleBetweenPrevAndNext.get("angle")) * angleBetweenCrankAndStick.get("sign");
    float f1r = radians(f1);
    // replace this with a rotation of p0 to f1
    PVector newP1 = new PVector(points[0].x, points[0].y);
    newP1.rotate(f1r);
    //PVector newP1 = new PVector(cos(f1r), sin(f1r));
    //newP1.mult(stickLength);
    newP1.add(points[0]);
    points[1].set(newP1.x, newP1.y);
    float f2 = radians(maxStickAngle * angleBetweenCrankAndStick.get("sign"));
    // replace this with rotation of p0 to f2
    //PVector newPrev = new PVector(cos(f2), sin(f2));
    PVector newPrev = new PVector(points[0].x, points[0].y);
    newPrev.rotate(f2);
    //newPrev.mult(stickLength);
    newPrev.add(points[0]);
    prevPoints[1].set(newPrev.x, newPrev.y);
    float newDist = dist(prevPoints[1].x, prevPoints[1].y, points[1].x, points[1].y);
    println("Reset, f1:", f1, "angleBetween:", angleBetweenPrevAndNext.get("angle"), "prevDist:", prevDist, "newDist:", newDist);
    println("prevPrev:", prevPrev, "prevPoint:", prevPoint);
    println("newPrev:", prevPoints[1], "newPoint:", points[1]);


    // just swap prev and next if passing angle boundary
    //if (false) {
    //  float angleDiff =  maxStickAngle - angleBetweenCrankAndStick;
    //  //angleDiff += -angleSign; // buffer of 2 degrees to prevent "bounce" during a reset
    //  //p1.rotate(radians(angleBetweenCrankAndStick * angleSign));
    //  //p1.mult(stickLength);
    //  PVector tempP = new PVector(prevPoints[1].x, prevPoints[1].y);
    //  //prevPoints[1].x = points[1].x;
    //  //prevPoints[1].y = points[1].y;
    //  points[1].set(prevPoints[1].x, prevPoints[1].y);

    //  p1.mult(stickLength);
    //  float felix = radians(maxStickAngle * angleSign);
    //  p1.rotate(felix);
    //  PVector check = new PVector(cos(felix), sin(felix));
    //  check.mult(stickLength);
    //  check.add(points[0]);
    //  prevPoints[1].set(check.x, check.y);
    //  //float newDist = dist(prevPoints[1].x, prevPoints[1].y, points[1].x, points[1].y);
    //  println("reset! tolerance:", stickAngleTolerance, "angleDiff:", angleDiff, "angleSign:", angleSign, 
    //    "angleBetweenCrankAndStick:", angleBetweenCrankAndStick, "prev:", prevPoints[1], 
    //    "prevDist:", prevDist, "newDist:", newDist, 
    //    "prevPrev", prevPrev, 
    //    "points1", points[1], "check:", check);
    //}
  }
}

void render() {

  //draw sticks
  stroke(255, 0, 0);

  translate(windowCenter.x, windowCenter.y);
  line(0, 0, points[0].x, points[0].y );
  line(points[0].x, points[0].y, points[1].x, points[1].y);


  //draw points
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
}
