# include "referenceline.hpp"

ReferenceLine::ReferenceLine(){
    read_module_planner_ini();
    readWayFromFile(file_path_);
}

void ReferenceLine::read_module_planner_ini()
{
    try
    {
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini("/home/dengjia/DengJia_ws/src/module_planner/src/module_planner.ini", pt);
        file_path_ =pt.get<std::string>("module_planner.way_file_path_");
    }
    catch (const boost::property_tree::ini_parser_error &e)
    {
        // 读取失败，输出错误信息
        std::cerr << "Error reading module_planner.ini: " << e.what() << std::endl;
    }
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