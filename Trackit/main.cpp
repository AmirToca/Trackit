#include <SFML\Graphics.hpp>

int main()
{


    sf::RenderWindow window(sf::VideoMode(512, 512), "gibilerinden");
    window.setFramerateLimit(60);

    sf::RectangleShape rect;
    sf::Vector2f rectanglePosition(100, 200);
    rect.setPosition(rectanglePosition);
    rect.setSize(sf::Vector2f(50, 50));

    float xVelocity = 3;
    float yVelocity = 5;

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed) window.close();
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) window.close();
        }


        if (rectanglePosition.x < 0 || rectanglePosition.x > 512 - 50) xVelocity *= -1;
        if (rectanglePosition.y < 0 || rectanglePosition.y > 512 - 50) yVelocity *= -1;

        //Physics
        rectanglePosition.x += xVelocity;
        rectanglePosition.y += yVelocity;
        rect.setPosition(rectanglePosition);



        //Render

        window.clear();
        window.draw(rect);
        window.display();
    }




    return 0;
}
