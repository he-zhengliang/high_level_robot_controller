#include "simulation/package_path.hpp"

namespace simulation {
    const std::string package_path::get_package_share_path(const std::string& package_name) {
        const char* env_string = std::getenv("CMAKE_PREFIX_PATH");
        size_t len = package_name.size();
        size_t start = 0;
        size_t stop = 0;
        
        while (true) {
            while (env_string[stop] != ':' && env_string[stop] != '\0') {
                stop++;
            }

            if (!package_name.compare(std::string(env_string + stop - len, len))) {
                return std::string(env_string + start, stop-start) + "/share/" + package_name + "/";
            }

            if (env_string[stop] == '\0') {
                std::cout << "Failed to find " << package_name << " in the environment. Ensure you sourced all the required packages\n"; 
                throw std::runtime_error::exception();
            } else {
                stop = start = stop+1;
            }
        }
    }
}