/* ================================================================================
 * Copyright: (C) 2020, SIRRL Social and Intelligent Robotics Research Laboratory, 
 *     University of Waterloo, All rights reserved.
 * 
 * Authors: 
 *     Austin Kothig <austin.kothig@uwaterloo.ca>
 * 
 * CopyPolicy: Released under the terms of the BSD 3-Clause License. 
 *     See the accompanying LICENSE file for details.
 * ================================================================================
 */

#ifndef HRI_PHYSIO_HELPERS_H
#define HRI_PHYSIO_HELPERS_H

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>

namespace hriPhysio {

    class InputParser;

    // TODO: REMOVE.
    void temp();

}


class hriPhysio::InputParser {
    /* ============================================================================
    **  Credit to users iain and 0x90
    **    https://stackoverflow.com/questions/865668/
    ** ============================================================================ */
public:
    InputParser(int &argc, char **argv) {
        for (int i = 1; i < argc; ++i) {
            this->tokens.push_back(std::string(argv[i]));
        }
    }

    const std::string& getCmdOption(const std::string &option) const {

        std::vector<std::string>::const_iterator itr;
        itr = std::find(this->tokens.begin(), this->tokens.end(), option);
        
        if (itr != this->tokens.end() && ++itr != this->tokens.end()){
            return *itr;
        }

        static const std::string empty_string("");
        return empty_string;
    }
    
    bool cmdOptionExists(const std::string &option) const {
        return std::find(
            this->tokens.begin(), this->tokens.end(), option
        ) != this->tokens.end();
    }
private:
    std::vector <std::string> tokens;
};

#endif /* HRI_PHYSIO_HELPERS_H */