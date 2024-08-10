#pragma once

namespace oplib::model{

    struct Node
    {
        /* data */
        int id;
        float x_coord, y_coord;
        double score;
        int t_opening, t_closing, t_service;
    };
    
}
