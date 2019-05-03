#include "SquarePositions.h"
#include <ctype.h>

float xOffset = 0.0;
float yOffset = 40.0;
float squareSize = 40.0;

bool getSquarePos(char hor, char vert, float &x, float &y)
{
    int horIdx = tolower(hor) - 'a';
    int vertIdx = vert - '1';

    if ((horIdx < 0) || (horIdx > 7) || (vertIdx < 0) || (vertIdx > 7))
    {
        return false;
    }

    x = xOffset + (7 - horIdx) * squareSize;
    y = yOffset + (7 - vertIdx) * squareSize;

    return true;
}
