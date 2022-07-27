#include <cmath>
#include <ostream>
#include <iostream>
#include "SocialForce.h"

#define PI 3.1415926

float L2Norm(vector<float>& x) {
    return sqrt(pow(x[0], 2) + pow(x[1], 2));
}

float Angel(vector<float>& x, vector<float>& y) {
    float cosr = (x[0] * y[0] + x[1] * y[1]) / (L2Norm(x) * L2Norm(y));
    return acos(cosr) * 180 / PI;
}

vector<float> Wall::nearestPoint(vector<float>& point) {
    
    float dx = position2[0] - position1[0];
    float dy = position2[1] - position1[1];

    float rx = position1[0] - point[0];
    float ry = position1[1] - point[1];
    
    float theta = -(rx * dx + ry * dy) / (dx * dx + dy * dy);

    if (theta < 0.0) return position1;
    if (theta > 1.0) return position2;

    return {theta * dx + position1[0], theta * dy + position1[1]};
}

vector<float> Agent::forceDriving(vector<Target>& targets) {
    const float T = 0.54F; // Relaxation Time

    // Desired Force
    vector<float> F_d(2, 0.0);

    for (auto& target : targets) {
        // Initialize Desired Direction
        vector<float> E_desired(2);
        E_desired[0] = target.position[0] - position[0];
        E_desired[1] = target.position[1] - position[1];

        // Calculate Force
        float E_desired_norm = L2Norm(E_desired);
        E_desired[0] = E_desired[0] / E_desired_norm;
        E_desired[1] = E_desired[1] / E_desired_norm;

        F_d[0] += (speed * E_desired[0] - velocity[0]) / T;
        F_d[1] += (speed * E_desired[1] - velocity[1]) / T;
    }

    return F_d;
}

vector<float> Agent::forceInteractAgent(vector<Agent>& agents) {
    const float lambda = 2.0;	// Weight reflecting relative importance of velocity vector against position vector
	const float gamma = 0.35F;	// Speed interaction
	const float n_prime = 3.0;	// Angular interaction
	const float n = 2.0;		// Angular intaraction
	const float A = 4.5;		// Modal parameter A

    // Social Force
    int K;
    float theta, f_v, f_theta;
    vector<float> F_ij(2, 0.0), E_ij(2), D_ij(2), T_ij(2);
    
    for (auto& agent : agents) {
        if (agent.id == id & agent.isBarrier == false) continue;
        
        // Initialize Distance
        E_ij[0] = agent.position[0] - position[0];
        E_ij[1] = agent.position[1] - position[1];

        // Skip Agents Which is Too Far Away
        float E_ij_norm = L2Norm(E_ij);
        if (E_ij_norm > 2) continue;
        E_ij[0] = E_ij[0] / E_ij_norm;
        E_ij[1] = E_ij[1] / E_ij_norm;

        // Calculate Paramters Between Agent i and j
        D_ij[0] = lambda * (velocity[0] - agent.velocity[0]) + E_ij[0];
        D_ij[1] = lambda * (velocity[1] - agent.velocity[1]) + E_ij[1];

        float B = gamma * L2Norm(D_ij);
        
        T_ij[0] = D_ij[0] / L2Norm(D_ij);
        T_ij[1] = D_ij[1] / L2Norm(D_ij);

        // Calculate Angle between T_ij and D_ij
        theta = Angel(T_ij, E_ij);
        K = (theta == 0) ? 0 : static_cast<int>(theta / abs(theta));

        f_v = -A * exp(-E_ij_norm / B - ((n_prime * B * theta) * (n_prime * B * theta)));
        f_theta = -A * K * exp(-E_ij_norm / B - ((n * B * theta) * (n * B * theta)));

        // Calculate Force
        F_ij[0] += f_v * T_ij[0] + f_theta * (-T_ij[1]);
        F_ij[1] += f_v * T_ij[1] + f_theta * T_ij[0];
    }

    return F_ij;
}

vector<float> Agent::forceInteractWall(vector<Wall>& walls) {
    const int a = 3;
	const float b = 0.1F;

    vector<float> E_iw;
    vector<float> F_iw(2, 0.0);

    for (auto& wall : walls) {
        E_iw = wall.nearestPoint(position);
        E_iw[0] = position[0] - E_iw[0];
        E_iw[1] = position[1] - E_iw[1];

        float E_iw_norm = L2Norm(E_iw);

        F_iw[0] += a * exp(-(E_iw_norm - radius) / b) * E_iw[0] / E_iw_norm;
        F_iw[1] += a * exp(-(E_iw_norm - radius) / b) * E_iw[1] / E_iw_norm;
    }

    return F_iw;
}
