#include "FlightComputer.h"
#include "Mat3x3.h"

using namespace std;

int main() {
    // FlightComputer fc;
    // fc.run();
    Mat3x3 defaultMatrix = Mat3x3();
    Mat3x3 customMatrix = Mat3x3(0, 1, 2, 3, 4, 5, 6, 7, 8);
    Mat3x3 identityMatrix = customMatrix.Identity();
    
    defaultMatrix.Print();
    cout << '\n' << endl;
    customMatrix.Print();
    cout << '\n' << endl;
    identityMatrix.Print();
    cout << '\n' << endl;

    Mat3x3 rotatedX = identityMatrix.RotateX(180);
    rotatedX.Print();
    
    Vec3 testVecX (1,0,0);
    Vec3 testVecY (0,1,0);
    Vec3 testVecZ (0,0,1);
    Vec3 testVecXYZ (1, 1, 1);

    testVecX = rotatedX.Multiply(testVecX);
    testVecY = rotatedX.Multiply(testVecY);
    testVecZ = rotatedX.Multiply(testVecZ);
    testVecXYZ = rotatedX.Multiply(testVecXYZ);
    
    cout << '\n' << endl;
    testVecX.Print();
    cout << '\n' << endl;
    testVecY.Print();
    cout << '\n' << endl;
    testVecZ.Print();
    cout << '\n' << endl;
    testVecXYZ.Print();
    cout << '\n' << endl;

    return 0;
}