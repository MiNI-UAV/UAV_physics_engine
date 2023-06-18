#include <Eigen/Dense>
#include "uav_params.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>
#include "rapidxml/rapidxml.hpp"

/// @brief Initialize default data
UAVparams::UAVparams() 
{
    name = "default";

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

void UAVparams::setMass(rapidxml::xml_node<> * interiaNode)
{
    for (rapidxml::xml_node<>* node = interiaNode->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"mass") == 0)
        {
            m = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"Ix") == 0)
        {
            Ix = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"Iy") == 0)
        {
            Iy = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"Iz") == 0)
        {
            Iz = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"Ixy") == 0)
        {
            Ixy = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"Ixz") == 0)
        {
            Ixz = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"Iyz") == 0)
        {
            Iyz = std::stod(node->value());
        }
    }
}

void UAVparams::setRotors(rapidxml::xml_node<> * rotorsNode)
{
    for (rapidxml::xml_node<>* node = rotorsNode->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"no") == 0)
        {
            noOfRotors = std::stod(node->value());
            rotorPos = new Eigen::Vector3d[noOfRotors];
            rotorDir = new int[noOfRotors];
            rotorTimeConstant.setZero(noOfRotors);
        }
        if(std::strcmp(node->name(),"forceCoff") == 0)
        {
            forceCoff = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"torqueCoff") == 0)
        {
            torqueCoff = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"positions") == 0)
        {
            int i = 0;
            for (rapidxml::xml_node<>* posNode = node->first_node(); posNode; i++, posNode = posNode->next_sibling()) 
            {
                double x,y,z;
                std::sscanf(posNode->value(),"%lf %lf %lf",&x,&y,&z);
                rotorPos[i] << x,y,z;
            }   
        }
        if(std::strcmp(node->name(),"direction") == 0)
        {
            int i = 0;
            for (rapidxml::xml_node<>* dirNode = node->first_node(); dirNode; i++, dirNode = dirNode->next_sibling()) 
            {
                rotorDir[i] = std::stoi(dirNode->value());
            } 
        }
        if(std::strcmp(node->name(),"timeConstants") == 0)
        {
            int i = 0;
            for (rapidxml::xml_node<>* timeNode = node->first_node(); timeNode; i++, timeNode = timeNode->next_sibling()) 
            {
                rotorTimeConstant(i) = std::stod(timeNode->value());
            }
        }
    }
}

void UAVparams::setAero(rapidxml::xml_node<> * aeroNode)
{
    for (rapidxml::xml_node<>* node = aeroNode->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"S") == 0)
        {
            S = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"d") == 0)
        {
            d = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"C") == 0)
        {
            std::sscanf(node->value(),"%lf %lf %lf %lf %lf %lf",&Ci[0],&Ci[1],&Ci[2],&Ci[3],&Ci[4],&Ci[5]);
        }
    }
}

UAVparams::UAVparams(std::string configFile)
{
    g = 9.81;
    ro = 1.204;

    if(!std::filesystem::exists(configFile))
    {  
        throw std::runtime_error("Config file not exist!");
    }
    std::cout << "Loading config" << std::endl;
    std::ifstream file(configFile);
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    rapidxml::xml_document<> doc;
    doc.parse<0>(&content[0]);
    rapidxml::xml_node<>* root = doc.first_node("params");
    for (rapidxml::xml_node<>* node = root->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"name") == 0)
        {
            name.assign(node->value(), node->value_size());
        }
        if(std::strcmp(node->name(),"ineria") == 0)
        {
            setMass(node);
        }
        if(std::strcmp(node->name(),"rotors") == 0)
        {
            setRotors(node);
        }
        if(std::strcmp(node->name(),"aero") == 0)
        {
            setAero(node);
        }
    }
}

UAVparams::~UAVparams()
{
    delete[] rotorPos;
    delete[] rotorDir;
}