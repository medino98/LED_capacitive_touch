#include <Arduino.h>
#define CIRCLE_SIZE 16

uint8_t circleData[CIRCLE_SIZE] = {0};
uint8_t circlePointer = 0;
uint16_t circleSum = 0;

void updateRollingAverage(uint8_t value)
{
    circleSum = circleSum - circleData[circlePointer] + value;
    circleData[circlePointer] = value;
    circlePointer++;
    if (circlePointer == CIRCLE_SIZE)
    {
        circlePointer = 0;
    }
}

uint8_t rollingAverage()
{
    return circleSum / CIRCLE_SIZE;
}
