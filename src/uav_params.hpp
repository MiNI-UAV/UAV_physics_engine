#pragma once
#include <Eigen/Dense>
#include <mutex>
#include <memory>
#include "rapidxml/rapidxml.hpp"


class Hinge
{
public:
    Hinge() = default;
    Hinge(Eigen::Vector3d axis, double max, double min, double trim);
    Hinge(const Hinge& old);
    Hinge& operator=(const Hinge& old);

    void updateValue(double newValue);
    const Eigen::Matrix3d getRot();

private:
    Eigen::Vector3d axis;
    double max;
    double min;

    std::mutex mtx;
    double value;
    Eigen::Matrix3d rot;
};


struct Rotor
{
    double forceCoff;
    double torqueCoff;
    Eigen::Vector3d position;
    Eigen::Vector3d axis;
    int direction;
    double timeConstant;
    double maxSpeed;
    int noOfHinges;
    Hinge hinges[2];
};

struct Jet
{
    Eigen::Vector3d position;
    Eigen::Vector3d axis;
    int phases;
    Eigen::VectorXd thrust;
    Eigen::VectorXd time;
    int noOfHinges;
    Hinge hinges[2];
};

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
        std::vector<Rotor> rotors;
        Eigen::VectorXd getRotorTimeContants();
        Eigen::VectorXd getRotorMaxSpeeds();

        //Jet params
        int noOfJets;
        std::vector<Jet> jets;

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
