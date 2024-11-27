#ifndef SIMULATION__PACKAGE_PATH_HPP
#define SIMULATION__PACKAGE_PATH_HPP

#include <string>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace simulation {
    class package_path {
    public:
        static const std::string get_package_share_path(const std::string& package_name);
    };
}

#endif