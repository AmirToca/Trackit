#include <SFML\Graphics.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>


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

int main()
{
    std::string filename = "Scenario2D.txt";
    std::vector<std::vector<double>> coordinates = readCoordinates(filename);
    int windowWidth = 800;
    int windowHeight = 800;
    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "gibilerinden");
    window.setFramerateLimit(60);



    int senarioStepCnt = 0;
    size_t scenarioTotalLength = coordinates.size();
    std::cout << scenarioTotalLength << std::endl;

    int rectWidth = 10;
    int rectHeight = 10;
    sf::RectangleShape rect;
    sf::Vector2f rectanglePosition(static_cast<float>(coordinates[senarioStepCnt][0]), static_cast<float>(coordinates[senarioStepCnt][1]));
    rect.setPosition(rectanglePosition);
    rect.setSize(sf::Vector2f(rectWidth, rectHeight));

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed) window.close();
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) window.close();
        }



        //Physics
        rectanglePosition.x = static_cast<float>(coordinates[senarioStepCnt][0]);
        rectanglePosition.y = static_cast<float>(coordinates[senarioStepCnt][1]);
        rect.setPosition(rectanglePosition);



        //Render
        if ((senarioStepCnt < scenarioTotalLength - 1) && (rectanglePosition.x > 0) && (rectanglePosition.x < windowWidth - rectWidth) &&
            (rectanglePosition.y > 0) && (rectanglePosition.y < windowHeight - rectHeight))
        {
            senarioStepCnt += 1;
            window.clear();
            window.draw(rect);
            window.display();
        }





    }

    




    return 0;
}
