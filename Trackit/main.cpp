#include <SFML\Graphics.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <cmath>


std::vector<std::vector<double>> readCoordinates(const std::string& filename) 
{
    std::vector<std::vector<double>> coordinates;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) 
    {
        std::istringstream iss(line);
        double x, y;
        if (iss >> x >> y) 
        {
            coordinates.push_back({ x, y });
        }
    }
    return coordinates;
}

std::vector<sf::Vector2f> convertToVector2f(const std::vector<std::vector<double>>& coordinates) {
    std::vector<sf::Vector2f> sfCoordinates;
    for (const auto& coord : coordinates) {
        sfCoordinates.emplace_back(static_cast<float>(coord[0]), static_cast<float>(coord[1]));
    }
    return sfCoordinates;
}



// Define matrix and vector types
typedef std::vector<std::vector<double>> Matrix;
typedef std::vector<double> Vector;

// Matrix and vector utility functions
Matrix matrixAdd(const Matrix& A, const Matrix& B);
Matrix matrixSubtract(const Matrix& A, const Matrix& B);
Matrix matrixMultiply(const Matrix& A, const Matrix& B);
Vector matrixVectorMultiply(const Matrix& A, const Vector& x);
Matrix transpose(const Matrix& A);
Matrix identityMatrix(size_t size);
Matrix inverse(const Matrix& A);
Vector vectorAdd(const Vector& A, const Vector& B);
Vector vectorSubtract(const Vector& A, const Vector& B);
Matrix multiplyMatrixByScalar(const Matrix& mat, double scalar);

class KalmanFilter {
public:
    KalmanFilter() {
        // Initial state
        x = { 0, 0, 5, 5 };  // [x, y, vx, vy]

        // State transition matrix
        F = {
            {1, 0, dt,  0},
            {0,  1, 0,  dt},
            {0,  0, 1, 0},
            {0,  0, 0,  1}
        };

        // Observation matrix
        H = {
            {1, 0, 0, 0},
            {0, 1, 0, 0}
        };
        
        // Process noise covariance
        Q = multiplyMatrixByScalar( {
            {std::pow(dt,4) / 4, 0, std::pow(dt,3) / 2, 0},
            {0, std::pow(dt,4) / 4, 0, std::pow(dt,3) / 2},
            {std::pow(dt,3) / 2, 0, std::pow(dt,2), 0},
            {0, std::pow(dt,3) / 2, 0, std::pow(dt,2)}
        }, q);

        // Measurement noise covariance
        R = {
            {1, 0},
            {0, 1}
        };

        // Estimation error covariance
        P = {
            {1, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        };
    }

    void predict() {
        x = matrixVectorMultiply(F, x);
        P = matrixAdd(matrixMultiply(F, matrixMultiply(P, transpose(F))), Q);
    }

    void update(const Vector& z) {
        Vector y = vectorSubtract(z, matrixVectorMultiply(H, x));
        Matrix S = matrixAdd(matrixMultiply(H, matrixMultiply(P, transpose(H))), R);
        Matrix K = matrixMultiply(P, matrixMultiply(transpose(H), inverse(S)));
        x = vectorAdd(x, matrixVectorMultiply(K, y));
        Matrix I = identityMatrix(P.size());
        P = matrixMultiply(matrixSubtract(I, matrixMultiply(K, H)), P);
    }

    Vector getState() {
        return x;
    }

private:
    double dt = 0.1; // Time step
    double q = 1; //Process noise multiplier
    Vector x; // State vector [x, y, vx, vy]
    Matrix F; // State transition matrix
    Matrix H; // Observation matrix
    Matrix Q; // Process noise covariance
    Matrix R; // Measurement noise covariance
    Matrix P; // Estimation error covariance
};

Matrix matrixAdd(const Matrix& A, const Matrix& B) {
    Matrix C(A.size(), Vector(A[0].size()));
    for (size_t i = 0; i < A.size(); ++i) {
        for (size_t j = 0; j < A[0].size(); ++j) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
    return C;
}

Matrix matrixSubtract(const Matrix& A, const Matrix& B) {
    Matrix C(A.size(), Vector(A[0].size()));
    for (size_t i = 0; i < A.size(); ++i) {
        for (size_t j = 0; j < A[0].size(); ++j) {
            C[i][j] = A[i][j] - B[i][j];
        }
    }
    return C;
}

Matrix matrixMultiply(const Matrix& A, const Matrix& B) {
    Matrix C(A.size(), Vector(B[0].size(), 0));
    for (size_t i = 0; i < A.size(); ++i) {
        for (size_t j = 0; j < B[0].size(); ++j) {
            for (size_t k = 0; k < A[0].size(); ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

Vector matrixVectorMultiply(const Matrix& A, const Vector& x) {
    Vector b(A.size(), 0);
    for (size_t i = 0; i < A.size(); ++i) {
        for (size_t j = 0; j < x.size(); ++j) {
            b[i] += A[i][j] * x[j];
        }
    }
    return b;
}

Matrix transpose(const Matrix& A) {
    Matrix B(A[0].size(), Vector(A.size()));
    for (size_t i = 0; i < A.size(); ++i) {
        for (size_t j = 0; j < A[0].size(); ++j) {
            B[j][i] = A[i][j];
        }
    }
    return B;
}

Matrix identityMatrix(size_t size) {
    Matrix I(size, Vector(size, 0));
    for (size_t i = 0; i < size; ++i) {
        I[i][i] = 1;
    }
    return I;
}

Matrix inverse(const Matrix& A) {
    // This is a simplified inverse function for 2x2 matrices
    Matrix B(2, Vector(2));
    double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    B[0][0] = A[1][1] / det;
    B[0][1] = -A[0][1] / det;
    B[1][0] = -A[1][0] / det;
    B[1][1] = A[0][0] / det;
    return B;
}

Vector vectorAdd(const Vector& A, const Vector& B) {
    Vector C(A.size());
    for (size_t i = 0; i < A.size(); ++i) {
        C[i] = A[i] + B[i];
    }
    return C;
}

Vector vectorSubtract(const Vector& A, const Vector& B) {
    Vector C(A.size());
    for (size_t i = 0; i < A.size(); ++i) {
        C[i] = A[i] - B[i];
    }
    return C;
}

Matrix multiplyMatrixByScalar(const Matrix& mat, double scalar) {
    // Create a new matrix with the same dimensions as the input matrix
    Matrix result(mat.size(), std::vector<double>(mat[0].size()));

    // Multiply each element by the scalar
    for (size_t i = 0; i < mat.size(); ++i) {
        for (size_t j = 0; j < mat[i].size(); ++j) {
            result[i][j] = mat[i][j] * scalar;
        }
    }

    return result;
}

int main()
{
    std::string filename = "Scenario2D.txt";
    std::vector<std::vector<double>> coordinates = readCoordinates(filename);
    int windowWidth = 800;
    int windowHeight = 800;
    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "gibilerinden");
    window.setFramerateLimit(60);

    KalmanFilter kf;
    
    int senarioStepCnt = 0;
    size_t scenarioTotalLength = coordinates.size();
    std::cout << scenarioTotalLength << std::endl;



    //Rectangle
    float rectWidth = 10.0;
    float rectHeight = 10.0;
    sf::RectangleShape rect;
    sf::Vector2f rectanglePosition(static_cast<float>(coordinates[senarioStepCnt][0]), static_cast<float>(coordinates[senarioStepCnt][1]));

    //rect.setPosition(rectanglePosition);
    rect.setSize(sf::Vector2f(rectWidth, rectHeight));
    rect.setOrigin(rectWidth / 2, rectHeight / 2);


    //Circle
    float circleRadius = 15;
    sf::CircleShape circle(circleRadius);
    //circle.setPosition(rectanglePosition.x - circle.getRadius(), rectanglePosition.y - circle.getRadius());

    // Set the outline thickness
    circle.setOutlineThickness(5); // Thickness of the outline

    // Set the outline color
    circle.setOutlineColor(sf::Color::Red); // Red outline

    // Set the fill color to transparent (hollow circle)
    circle.setFillColor(sf::Color::Transparent); // Transparent fill

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed) window.close();
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) window.close();
        }

        Vector z = { coordinates[senarioStepCnt][0] , coordinates[senarioStepCnt][1] };

        kf.predict();
        kf.update(z);

        Vector state = kf.getState();



        //Physics
        rectanglePosition.x = static_cast<float>(z[0]);
        rectanglePosition.y = static_cast<float>(z[1]);
        rect.setPosition(rectanglePosition);
        //Rectangle will be in the center of the circle.
        sf::Vector2f circleCenter(state[0] - circle.getRadius(), state[1] - circle.getRadius());
        circle.setPosition(circleCenter);


        //Render
        if ((senarioStepCnt < scenarioTotalLength - 1) && (rectanglePosition.x > 0) && (rectanglePosition.x < windowWidth - rectWidth) &&
            (rectanglePosition.y > 0) && (rectanglePosition.y < windowHeight - rectHeight))
        {
            senarioStepCnt += 1;
            window.clear();
            window.draw(rect);
            window.draw(circle);
            window.display();
        }





    }

    




    return 0;
}
