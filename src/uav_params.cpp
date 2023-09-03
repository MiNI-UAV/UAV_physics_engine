#include <Eigen/Dense>
#include "uav_params.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <mutex>
#include "rapidxml/rapidxml.hpp"

/// @brief Initialize default data
UAVparams::UAVparams() 
{
    name = "default";

    m = 5;
    Ix = 10;
    Iy = 11;
    Iz = 12;
    Ixy = 1;
    Ixz = 2;
    Iyz = 3;

    noOfRotors = 0;

    S = 0.1;
    d = 0.001;

    if(singleton != nullptr)
    {
        std::cerr << "Only one instance of UAVParams should exist";
        return;
    }
    singleton = this;
}

UAVparams* UAVparams::singleton = nullptr;

Eigen::VectorXd UAVparams::getRotorTimeContants()
{
    auto vec = Eigen::VectorXd(noOfRotors);
    for (int i = 0; i < noOfRotors; i++)
    {
        vec(i) = rotors[i].timeConstant;
    }
    return vec;
}

Eigen::VectorXd UAVparams::getRotorMaxSpeeds()
{
    Eigen::VectorXd vec;
    vec.setZero(noOfRotors);
    for (int i = 0; i < noOfRotors; i++)
    {
        vec(i) = rotors[i].maxSpeed;
    }
    return vec;
}

UAVparams *UAVparams::getSingleton()
{
    return singleton;
}

void UAVparams::setMass(rapidxml::xml_node<> *interiaNode)
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

void parseHinge(rapidxml::xml_node<>* hingeNode, Hinge* hinge)
{
    Eigen::Vector3d axis(1.0,0.0,0.0);
    double max = 0.0;
    double min = 0.0;
    double trim = 0.0;
    for (rapidxml::xml_node<>* node = hingeNode->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"axis") == 0)
        {
            double x,y,z;
            std::sscanf(node->value(),"%lf %lf %lf",&x,&y,&z);
            axis << x,y,z;
        }

        if(std::strcmp(node->name(),"min") == 0)
        {
            min = std::stod(node->value());
        }

        if(std::strcmp(node->name(),"max") == 0)
        {
            max = std::stod(node->value());
        }

        if(std::strcmp(node->name(),"trim") == 0)
        {
            trim = std::stod(node->value());
        }

    }
    *hinge = Hinge(axis,max,min,trim);
}

void UAVparams::setRotors(rapidxml::xml_node<> * rotorsNode)
{
    noOfRotors = std::stoi(rotorsNode->first_attribute()->value());

    for (rapidxml::xml_node<>* rotorNode = rotorsNode->first_node(); rotorNode; rotorNode = rotorNode->next_sibling()) 
    {
        if(std::strcmp(rotorNode->name(),"rotor") != 0) continue;

        Rotor rotor;

        for(rapidxml::xml_node<>* node = rotorNode->first_node(); node; node = node->next_sibling())
        {


            if(std::strcmp(node->name(),"forceCoff") == 0)
            {
                rotor.forceCoff = std::stod(node->value());
            }

            if(std::strcmp(node->name(),"torqueCoff") == 0)
            {
                rotor.torqueCoff = std::stod(node->value());
            }

            if(std::strcmp(node->name(),"position") == 0)
            {
                double x,y,z;
                std::sscanf(node->value(),"%lf %lf %lf",&x,&y,&z);
                rotor.position << x,y,z;
            }

            if(std::strcmp(node->name(),"axis") == 0)
            {
                double x,y,z;
                std::sscanf(node->value(),"%lf %lf %lf",&x,&y,&z);
                rotor.axis << x,y,z;
            }

            if(std::strcmp(node->name(),"direction") == 0)
            {
                rotor.direction = std::stoi(node->value());
            }

            if(std::strcmp(node->name(),"timeConstant") == 0)
            {
                rotor.timeConstant = std::stod(node->value());
            }

            if(std::strcmp(node->name(),"maxSpeed") == 0)
            {
                rotor.maxSpeed = std::stod(node->value());
            }

            if(std::strcmp(node->name(),"hinges") == 0)
            {
                rotor.noOfHinges = std::stod(node->first_attribute()->value());
                int i = 0;
                for(rapidxml::xml_node<>* hingeNode = node->first_node(); hingeNode && i < rotor.noOfHinges; hingeNode = hingeNode->next_sibling(), i++)
                {
                    parseHinge(hingeNode, &rotor.hinges[i]);
                }
            }
        }
        rotors.push_back(rotor);
    }
}

Eigen::VectorXd parseVectorXd(std::string str, int noOfElem)
{
    Eigen::VectorXd res;
    res.setZero(noOfElem);
    std::istringstream f(str);
    std::string s;
    int i;
    for (i = 0; i < noOfElem; i++)
    {
        if(!getline(f, s, ' '))
        {
            std::cerr << "Parse VectorXd error" << std::endl;
            break;
        }
        res(i) = std::stod(s);
    }
    if(i != noOfElem)
    {
        std::cerr << "Parse VectorXd error" << std::endl;
        return Eigen::VectorXd(0);
    }
    return res;
}

void UAVparams::setJets(rapidxml::xml_node<> * jetsNode)
{
    noOfJets = std::stoi(jetsNode->first_attribute()->value());

    for (rapidxml::xml_node<>* jetNode = jetsNode->first_node(); jetNode; jetNode = jetNode->next_sibling()) 
    {
        if(std::strcmp(jetNode->name(),"jet") != 0) continue;

        Jet jet;

        for(rapidxml::xml_node<>* node = jetNode->first_node(); node; node = node->next_sibling())
        {
            if(std::strcmp(node->name(),"position") == 0)
            {
                double x,y,z;
                std::sscanf(node->value(),"%lf %lf %lf",&x,&y,&z);
                jet.position << x,y,z;
            }

            if(std::strcmp(node->name(),"axis") == 0)
            {
                double x,y,z;
                std::sscanf(node->value(),"%lf %lf %lf",&x,&y,&z);
                jet.axis << x,y,z;
            }

            if(std::strcmp(node->name(),"phases") == 0)
            {
                jet.phases = std::stoi(node->value());
            }

            if(std::strcmp(node->name(),"thrust") == 0)
            {
                jet.thrust = parseVectorXd(node->value(),jet.phases);
            }

            if(std::strcmp(node->name(),"time") == 0)
            {
                jet.time = parseVectorXd(node->value(),jet.phases);
            }

            if(std::strcmp(node->name(),"hinges") == 0)
            {
                jet.noOfHinges = std::stod(node->first_attribute()->value());
                int i = 0;
                for(rapidxml::xml_node<>* hingeNode = node->first_node(); hingeNode && i < jet.noOfHinges; hingeNode = hingeNode->next_sibling(), i++)
                {
                    parseHinge(hingeNode, &jet.hinges[i]);
                }
            }
        }
        jets.push_back(jet);
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

void UAVparams::loadConfig(std::string configFile)
{
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
        if(std::strcmp(node->name(),"jets") == 0)
        {
            setJets(node);
        }
        if(std::strcmp(node->name(),"aero") == 0)
        {
            setAero(node);
        }
    }
}

UAVparams::~UAVparams()
{
    singleton = nullptr;
}

Hinge::Hinge(Eigen::Vector3d axis, double max, double min, double trim):
    axis{axis}, max{max}, min{min}, value{trim}
{
    updateValue(value);
}

Hinge::Hinge(const Hinge &old)
{
    this->axis = old.axis;
    this->min = old.min;
    this->max = old.max;
    updateValue(old.value);
}

Hinge &Hinge::operator=(const Hinge &old)
{
    this->axis = old.axis;
    this->min = old.min;
    this->max = old.max;
    updateValue(old.value);
    return *this;
}

void Hinge::updateValue(double newValue) 
{
    static const Eigen::Matrix3d crossProductMatrix = axis.asSkewSymmetric().toDenseMatrix();
    static const Eigen::Matrix3d outerProduct = axis * axis.transpose();
    std::scoped_lock lck(mtx);
    value = newValue;
    rot = cos(value) * Eigen::Matrix3d::Identity() 
        + sin(value) * crossProductMatrix
        + (1 - cos(value)) * outerProduct;
}

const Eigen::Matrix3d Hinge::getRot()
{
   std::scoped_lock lck(mtx);
   return rot;
}
