#pragma once

#include <Arduino.h>
#include "Config.h"
#include "Solver.h"
#include "Navigator.h"

#define ROUTER_PATH_BUFFER_SIZE 256

char router_path_buffer[ROUTER_PATH_BUFFER_SIZE];
size_t router_path_index = 0;

Vec2 router_look = Vec2{0, 0};
char router_state = 0;

void router_init()
{
    for (int i = 0; i < ROUTER_PATH_BUFFER_SIZE; i++)
    {
        router_path_buffer[i] = 0;
    }
    router_path_index = 0;

    router_look = nav_get_pos();

    char symbs[] = {'E', 0, 'N', 0, 'W', 0, 'S', 0};
    router_state = symbs[nav_get_sigma()];
}

void router_add_step(char step)
{
    if (router_path_index < ROUTER_PATH_BUFFER_SIZE)
    {
        router_path_buffer[router_path_index] = step;
        router_path_index++;
    }
}

void router_tick()
{
    for (size_t i = 0;
         i < ROUTER_PATH_BUFFER_SIZE &&
         router_state != 'G';
         i++)
    {
        char input = solver_get_where_from(router_look);

        switch (router_state)
        {
        case 'N':
            if (input == 'N')
                router_add_step('F');
            if (input == 'E')
                router_add_step('R');
            if (input == 'W')
                router_add_step('L');
            if (input == 'S')
                router_add_step('I');
            if (input == 'G')
                router_add_step('S');
            break;
        case 'W':
            if (input == 'W')
                router_add_step('F');
            if (input == 'N')
                router_add_step('R');
            if (input == 'S')
                router_add_step('L');
            if (input == 'E')
                router_add_step('I');
            if (input == 'G')
                router_add_step('S');
            break;
        case 'S':
            if (input == 'S')
                router_add_step('F');
            if (input == 'W')
                router_add_step('R');
            if (input == 'E')
                router_add_step('L');
            if (input == 'N')
                router_add_step('I');
            if (input == 'G')
                router_add_step('S');
            break;
        case 'E':
            if (input == 'E')
                router_add_step('F');
            if (input == 'S')
                router_add_step('R');
            if (input == 'N')
                router_add_step('L');
            if (input == 'W')
                router_add_step('I');
            if (input == 'G')
                router_add_step('S');
            break;
        case 'G':
            break;

        default:
            break;
        }

        router_state = input;

        if (router_state == 'N')
            router_look.y -= 1;
        else if (router_state == 'S')
            router_look.y += 1;
        else if (router_state == 'E')
            router_look.x += 1;
        else if (router_state == 'W')
            router_look.x -= 1;
    }
}
