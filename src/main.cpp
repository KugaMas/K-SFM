#include <stdio.h>
#include <ostream>
#include <iostream>

#include "SocialForce.h"

using namespace std;

int main() {
    vector<float> a = {1, 1};
    vector<float> b = {2, 2};
    Wall x(a, b);
    vector<float> c = {1, 2};
    Agent y(c);

    vector<float> dist = x.nearestPoint(y.position);
    cout << dist[0] << ' ' << dist[1] << endl;
    return 0;
}
