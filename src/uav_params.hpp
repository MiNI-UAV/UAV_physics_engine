#pragma once
#include <Eigen/Dense>
#include <mutex>
#include <memory>
#include "rapidxml/rapidxml.hpp"

#include "components/drive.hpp"
#include "components/control_surfaces.hpp"

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
        Eigen::VectorXd getRotorTimeContants() const;
        Eigen::VectorXd getRotorMaxSpeeds() const;

        //Jet params
        int noOfJets;
        std::unique_ptr<Jet[]> jets;

        //Surface params
        ControlSurfaces surfaces;

        //Aerodynamic params
        double S, d;
        double Ci[6] = {1.0,1.0,1.0,0.0,0.0,0.0};

        const static UAVparams* getSingleton();

    private:
        
        void setMass(rapidxml::xml_node<> * interiaNode);
        void setRotors(rapidxml::xml_node<> * rotorsNode);
        void setJets(rapidxml::xml_node<> * rotorsNode);
        void setAero(rapidxml::xml_node<> * aeroNode);
        void setControlSurface(rapidxml::xml_node<> * surfaceNode);

        static UAVparams* singleton;

};
