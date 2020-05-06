#pragma once

struct WeatherMessage
{
    float temperature;
    float pressure;
    float humidity;
};

void create_weather_msg(char *msg, struct WeatherMessage *msg_struct);