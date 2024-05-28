#ifndef REFERENCE_LINE_HPP
#define REFERENCE_LINE_HPP

#include <fstream>
#include <sstream>
#include "common.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <iostream>

class ReferenceLine{
public:
    ReferenceLine();

    void GenerateReferenceLine(Traj &Traj_);

    ~ReferenceLine() = default;
private:
    Way way_;
    std::string file_path_;
    void read_module_planner_ini();
    void readWayFromFile(const std::string& filename);
    bool parseLine(const std::string& line, double& x, double& y);
};

#endif // REFERENCE_LINE_HPP