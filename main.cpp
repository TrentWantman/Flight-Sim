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

    Mat3x3 rotatedX = Mat3x3::RotateX(180);
    rotatedX.Print();
    cout << '\n' << endl;
    printf("Determinant: %f\n", rotatedX.Determinant());
    cout << '\n' << endl;

    Mat3x3 rotX = Mat3x3::RotateX(90);
    Mat3x3 rotY = Mat3x3::RotateY(90);
    Mat3x3 combined = rotY.Multiply(rotX);
    combined.Print();
    cout << '\n' << endl;


    Vec3 v(1, 0, 0);

    // Step by step
    Vec3 step1 = rotX.Multiply(v);
    Vec3 step2 = rotY.Multiply(step1);
    step2.Print();
    cout << '\n' << endl;


    // Combined
    Vec3 combined_result = combined.Multiply(v);
    combined_result.Print();
    cout << '\n' << endl;

    return 0;
}