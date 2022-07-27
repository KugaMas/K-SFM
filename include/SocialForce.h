#ifndef SOCIAL_FORCE_H
#define SOCIAL_FORCE_H

#include <vector>
using namespace std;

class Target {
public:
    vector<float> position;

    Target(){}
    Target(vector<float> x) : position(x) {}
};

class Wall {
public:
    vector<float> position1;
    vector<float> position2;

    Wall(){}
    Wall(vector<float> x1, vector<float> x2) : position1(x1), position2(x2) {}

    vector<float> nearestPoint(vector<float>& point);
};

class Agent {
private:
    int id;
    bool isBarrier = false;
public:
    float speed;             // Agent Desired Velocity
    float radius;            // Agent Round
    vector<float> velocity;  // Agent Acrtual Velocity
    vector<float> position;  // Agent Coordinate

    Agent(vector<float> x) : position(x), radius(0.2) {}

    vector<float> forceDriving(vector<Target>& targets);           // self driving force
    vector<float> forceInteractWall(vector<Wall>& walls);       // obstacle force
    vector<float> forceInteractAgent(vector<Agent>& agents);    // pedestrian force
};

#endif