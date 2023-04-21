#include <Eigen/Dense>
#include "uav_params.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>
#include "rapidxml/rapidxml.hpp"

/// @brief Initialize default data
UAVparams::UAVparams() 
{
    g = 9.81;
    ro = 1.204;

    m = 5;
    Ix = 10;
    Iy = 11;
    Iz = 12;
    Ixy = 1;
    Ixz = 2;
    Iyz = 3;

    forceCoff = 1.0;
    torqueCoff = 1.0;
    noOfRotors = 4;
    rotorPos = new Eigen::Vector3d[noOfRotors]{{ 0.1, 0.1, 0.0},
                                               {-0.1, 0.1, 0.0},
                                               {-0.1,-0.1, 0.0},
                                               { 0.1,-0.1, 0.0}};
    rotorDir = new int[noOfRotors] {1,-1, 1,-1};
    rotorTimeConstant.setConstant(noOfRotors,0.05);

    S = 0.1;
    d = 0.001;
}

UAVparams::UAVparams(const char* configFile) : UAVparams()
{
    if(std::filesystem::exists(configFile))
    {  
        std::cout << "Loading config" << std::endl;
        std::ifstream file(configFile);
        std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        // Parse the XML file using RapidXML
        rapidxml::xml_document<> doc;
        doc.parse<0>(&content[0]);

        rapidxml::xml_node<>* root = doc.first_node("root");

        // Iterate over the child nodes
        for (rapidxml::xml_node<>* node = root->first_node(); node; node = node->next_sibling()) {
            // Access the node's name and value
            std::cout << "Node name: " << node->name() << std::endl;
            std::cout << "Node value: " << node->value() << std::endl;

            // Iterate over the node's attributes
            for (rapidxml::xml_attribute<>* attr = node->first_attribute(); attr; attr = attr->next_attribute()) {
                // Access the attribute's name and value
                std::cout << "Attribute name: " << attr->name() << std::endl;
                std::cout << "Attribute value: " << attr->value() << std::endl;
            }
        }
    }
    else
    {
        std::cout << "Config file not exist!" << std::endl;
    }
}

UAVparams::~UAVparams()
{
    delete[] rotorPos;
    delete[] rotorDir;
}