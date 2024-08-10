#pragma once
#include <algorithm>
#include <string>

namespace oplib::utils{

    class ArgParser
    {
    public:
        char ** begin;
        char ** end;
        ArgParser(char** begin, char ** end): begin(begin), end(end){};
        ~ArgParser()=default;

        inline char* getCmdOption(const std::string & option)
        {
            char ** itr = std::find(begin, end, option);
            if (itr != end && ++itr != end)
            {
                return *itr;
            }
            return 0;
        }

        inline bool cmdOptionExists(const std::string& option)
        {
            return std::find(begin, end, option) != end;
        }

        inline int getCmdInt(const std::string & option, int default_value){
            char * value = getCmdOption(option);
            return value ? atoi(value) : default_value;
        }

        inline int getCmdDouble(const std::string & option, double default_value){
            char * value = getCmdOption(option);
            return value ? atof(value) : default_value;
        }


    };

}