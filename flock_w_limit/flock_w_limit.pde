import peasy.*;
int AXES_DISTANCE = 1000;
float MULT_FACTOR = 10;
float MAX_FORCE = 0.2 * MULT_FACTOR;
float MAX_SPEED = 2 * MULT_FACTOR;
float _RADIUS = 400;
int LOWERBOUND = -(AXES_DISTANCE - 50);
int UPPERBOUND = AXES_DISTANCE - 50;
PeasyCam camera;
Flock flock;

PVector generateRandomPosition() {
  float x = random(LOWERBOUND, UPPERBOUND);
  float y = random(LOWERBOUND, UPPERBOUND);
  float z = random(LOWERBOUND, UPPERBOUND);
  return new PVector(x, y, z);
}

void setup() {
  size(1024, 1024, P3D);
  camera = new PeasyCam(this, AXES_DISTANCE*3.5);
  flock = new Flock(500);
}

void draw() {
  background(255);
  draw_environment();
  flock.step();
  flock.display();
}
void draw_environment() { 
  noFill();
  //X Axes - Red
  stroke(255, 0, 0);
  line(0, 0, AXES_DISTANCE, 0);
  //Y Axes - Green
  stroke(0, 255, 0);
  line(0, 0, 0, AXES_DISTANCE);
  //Z Axes - Blue
  stroke(0, 0, 255);
  line(0, 0, 0, 0, 0, -AXES_DISTANCE);
  //Fish Tank
  stroke(0);
  box(AXES_DISTANCE*2, AXES_DISTANCE*2, AXES_DISTANCE*2);
}

class Boid {
  PVector location;
  PVector velocity;
  PVector acceleration;

  Boid(float x, float y, float z) {
    location = new PVector(x, y, z);
    acceleration = new PVector(0, 0, 0);
    velocity = PVector.random3D();
  }
  PVector steer(PVector target) {
    PVector desired = target.sub(location);
    desired.normalize();
    desired.mult(MAX_SPEED);
    PVector steer = desired.sub(velocity);
    steer.limit(MAX_FORCE);
    return steer;
  }
  void applyForce(PVector force) {
    acceleration.add(force);
  }  
  void step() {
    boundaries();
    velocity.add(acceleration);
    velocity.limit(MAX_SPEED);
    location.add(velocity);
    acceleration.mult(0);
  }
  void boundaries() {
    if (location.x > UPPERBOUND || location.x < LOWERBOUND) {
      velocity.x = -velocity.x;
    }
    if (location.y > UPPERBOUND || location.y < LOWERBOUND) {
      velocity.y = -velocity.y;
    }
    if (location.z > UPPERBOUND || location.z < LOWERBOUND) {
      velocity.z = -velocity.z;
    }
  }
  void display() {  
    fill(50);
    pushMatrix();
    translate(location.x, location.y, location.z);
    box(8);
    popMatrix();
  }
  PVector cohesion(Boid[] boids) {
    PVector acc = new PVector();
    int count = 0;
    for (int i = 0; i < boids.length; i++) {
      float d = PVector.dist(this.location, boids[i].location);
      if (d <= _RADIUS && d != 0) {
        acc.add(boids[i].location);
        count++;
      }
    }
    if (count == 0) {
      return location;
    }
    acc.div(count);
    return acc;
  }
  PVector separation(Boid[] boids) {
    PVector sep = new PVector();
    int count = 0;
    for (int i = 0; i < boids.length; i++) {
      float dist = location.dist(boids[i].location);
      if (dist <= (_RADIUS/5) && dist > 0.001) {
        PVector dv = (this.location.sub(boids[i].location));
        sep.add(dv);
        count++;
      }
    }
    if (count == 0) {
      return location;
    }
    sep.div(count);
    return sep;
  }
}
class Flock {
  Boid[] boids;
  Flock(int size) {
    boids = new Boid[size];
    for (int i = 0; i < boids.length; i++) {
      PVector r = generateRandomPosition();
      boids[i] = new Boid(r.x, r.y, r.z);
    }
  }
  void step() {
    for (int i = 0; i < boids.length; i++) {
      PVector cohesion = boids[i].cohesion(boids);
      PVector steerForce = boids[i].steer(cohesion);
      boids[i].applyForce(steerForce);
      boids[i].step();
    }
  }
  void display() {
    for (int i = 0; i < boids.length; i++) {
      boids[i].display();
    }
  }
}