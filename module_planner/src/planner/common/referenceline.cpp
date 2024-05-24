# include "referenceline.hpp"

ReferenceLine::ReferenceLine() : file_path_("/home/dengjia/DengJia_ws/src/way_network/WayPoints/waypoints_1.txt"){
    readWayFromFile(file_path_);
}

void ReferenceLine::GenerateReferenceLine(Traj &Traj_){
    Traj_ = way_.toTraj(); 
}



void ReferenceLine::readWayFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + filename);
    }
    std::string line;
    while (std::getline(file, line)) {
        double x, y;
        if (parseLine(line, x, y)) {
            way_.way_points.emplace_back(x, y);
        }
    }
    file.close();
}

bool ReferenceLine::parseLine(const std::string& line, double& x, double& y) {
    std::istringstream iss(line);
    if (!(iss >> x >> y)) {
        return false;
    }
    return true;
}