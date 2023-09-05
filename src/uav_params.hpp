#pragma once
#include <Eigen/Dense>
#include <mutex>
#include <memory>
#include "rapidxml/rapidxml.hpp"

#include "drives/drive.hpp"

struct UAVparams
{
    public:
        UAVparams();
        ~UAVparams();
        void loadConfig(std::string configFile);

        std::string name;

        bool instantRun;


        //Mass params
        double m;
        double Ix;
        double Iy;
        double Iz;
        double Ixy;
        double Ixz;
        double Iyz;

        //Rotor params
        int noOfRotors;
        std::unique_ptr<Rotor[]> rotors;
        Eigen::VectorXd getRotorTimeContants();
        Eigen::VectorXd getRotorMaxSpeeds();

        //Jet params
        int noOfJets;
        std::unique_ptr<Jet[]> jets;

        //Aerodynamic params
        double S, d;
        double Ci[6] = {1.0,1.0,1.0,0.0,0.0,0.0};

        static UAVparams* getSingleton();

    private:
        
        void setMass(rapidxml::xml_node<> * interiaNode);
        void setRotors(rapidxml::xml_node<> * rotorsNode);
        void setJets(rapidxml::xml_node<> * rotorsNode);
        void setAero(rapidxml::xml_node<> * aeroNode);

        static UAVparams* singleton;

};
