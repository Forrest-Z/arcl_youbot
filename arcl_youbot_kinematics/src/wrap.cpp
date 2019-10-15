#include "arcl_youbot_kinematics/arm_dynamics.h"
#include "arcl_youbot_kinematics/arm_kinematics.h"
#include "arcl_youbot_kinematics/constants.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>


namespace py = pybind11;
namespace ayk = arcl_youbot_kinematics;


PYBIND11_MODULE(wrap, m) {
    m.doc() = "pybind11 arcl_youbot_kinematics"; // optional module docstring


    // ---------- arm_kinematics.h ----------
    // non-class functions
    m.def("getMinJointPositions", &ayk::getMinJointPositions);
    m.def("getMaxJointPositions", &ayk::getMaxJointPositions);
    m.def("max", &ayk::max);
    m.def("min", &ayk::min);
    m.def("abs", &ayk::abs);

    // GenericVector
    py::class_<ayk::GenericVector>(m, "GenericVector")
        .def(py::init<>())
        .def("setValues", py::overload_cast<const std::vector<double> &>(&ayk::GenericVector::setValues))
        // Use below after reading https://pybind11.readthedocs.io/en/stable/faq.html#limitations-involving-reference-arguments
        .def("setValues", py::overload_cast<const double *>(&ayk::GenericVector::setValues))
        .def("printValues", &ayk::GenericVector::printValues)
        .def("max", &ayk::GenericVector::max)
        .def("min", &ayk::GenericVector::min)
        .def("abs", &ayk::GenericVector::abs)
        .def("isValid", &ayk::GenericVector::isValid)
        .def("isZero", &ayk::GenericVector::isZero)
        .def(py::self += py::self)
        .def(py::self -= py::self)
        .def(py::self *= double())
        .def(py::self /= double())
        .def(py::self *= py::self)
        .def(py::self /= py::self)
        .def(py::self - py::self)
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(double() * py::self)
        .def(py::self * double())
        .def(py::self / double());

    // JointVector
    py::class_<ayk::JointVector, ayk::GenericVector>(m, "JointVector")
        .def(py::init<>())
        .def(py::init<const std::vector<double>>())
        .def("q1", &ayk::JointVector::q1)
        .def("q2", &ayk::JointVector::q2)
        .def("q3", &ayk::JointVector::q3)
        .def("q4", &ayk::JointVector::q4)
        .def("q5", &ayk::JointVector::q5)
        .def("setQ1", &ayk::JointVector::setQ1)
        .def("setQ2", &ayk::JointVector::setQ2)
        .def("setQ3", &ayk::JointVector::setQ3)
        .def("setQ4", &ayk::JointVector::setQ4)
        .def("setQ5", &ayk::JointVector::setQ5);

    // JointPosition
    py::class_<ayk::JointPosition, ayk::JointVector>(m, "JointPosition")
        .def(py::init<>())
        .def(py::init<const std::vector<double> &>())
        .def("toCylindric", &ayk::JointPosition::toCylindric)
        .def("toCartesian", &ayk::JointPosition::toCartesian)
        .def("addOffset", &ayk::JointPosition::addOffset)
        .def("subtractOffset", &ayk::JointPosition::subtractOffset)
        .def("isReachable", &ayk::JointPosition::isReachable);

    // JointVelocitytoJointspace
    py::class_<ayk::JointVelocity, ayk::JointVector>(m, "JointVelocity")
        .def(py::init<>())
        .def(py::init<const std::vector<double> &>())
        .def("toCylindric", &ayk::JointVelocity::toCylindric)
        .def("toCartesian", &ayk::JointVelocity::toCartesian);

    // JointAcceleration
    py::class_<ayk::JointAcceleration, ayk::JointVector>(m, "JointAcceleration")
        .def(py::init<>())
        .def(py::init<const std::vector<double> &>());

    // CylindricVector
    py::class_<ayk::CylindricVector, ayk::GenericVector>(m, "CylindricVector")
        .def(py::init<>())
        .def(py::init<const std::vector<double>>())
        .def("q1", &ayk::CylindricVector::q1)
        .def("r", &ayk::CylindricVector::r)
        .def("z", &ayk::CylindricVector::z)
        .def("theta", &ayk::CylindricVector::theta)
        .def("q5", &ayk::CylindricVector::q5)
        .def("setQ1", &ayk::CylindricVector::setQ1)
        .def("setR", &ayk::CylindricVector::setR)
        .def("setZ", &ayk::CylindricVector::setZ)
        .def("setTheta", &ayk::CylindricVector::setTheta)
        .def("setQ5", &ayk::CylindricVector::setQ5);

    // CylindricPosition
    py::class_<ayk::CylindricPosition, ayk::CylindricVector>(m, "CylindricPosition")
        .def(py::init<>())
        .def(py::init<const std::vector<double>>())
        .def("toJointspace", py::overload_cast<const ayk::JointPosition &, bool>(&ayk::CylindricPosition::toJointspace, py::const_), 
            py::arg(), py::arg("same_config")=false)
        .def("toJointspace", py::overload_cast<bool>(&ayk::CylindricPosition::toJointspace, py::const_))
        .def("toJointspace", py::overload_cast<>(&ayk::CylindricPosition::toJointspace, py::const_))
        .def("toCartesian", &ayk::CylindricPosition::toCartesian)
        .def("isReachable", &ayk::CylindricPosition::isReachable);

    // CylindricVelocity
    py::class_<ayk::CylindricVelocity, ayk::CylindricVector>(m, "CylindricVelocity")
        .def(py::init<>())
        .def(py::init<const std::vector<double>>())
        .def("toCartesian", &ayk::CylindricVelocity::toCartesian)
        .def("toJointspace", &ayk::CylindricVelocity::toJointspace);

    // CylindricAcceleration
    py::class_<ayk::CylindricAcceleration, ayk::CylindricVector>(m, "CylindricAcceleration")
        .def(py::init<>())
        .def(py::init<const std::vector<double>>())
        .def("toCartesian", &ayk::CylindricAcceleration::toCartesian);

    // CartesianVector
    py::class_<ayk::CartesianVector, ayk::GenericVector>(m, "CartesianVector")
        .def(py::init<>())
        .def(py::init<const std::vector<double>>())
        .def("x", &ayk::CartesianVector::x)
        .def("y", &ayk::CartesianVector::y)
        .def("z", &ayk::CartesianVector::z)
        .def("theta", &ayk::CartesianVector::theta)
        .def("q5", &ayk::CartesianVector::q5)
        .def("setX", &ayk::CartesianVector::setX)
        .def("setY", &ayk::CartesianVector::setY)
        .def("setZ", &ayk::CartesianVector::setZ)
        .def("setTheta", &ayk::CartesianVector::setTheta)
        .def("setQ5", &ayk::CartesianVector::setQ5);

    // CartesianPosition
    py::class_<ayk::CartesianPosition, ayk::CartesianVector>(m, "CartesianPosition")
        .def(py::init<>())
        .def(py::init<const std::vector<double>>())
        .def("toJointspace", py::overload_cast<const ayk::JointPosition &, bool>(&ayk::CartesianPosition::toJointspace, py::const_), 
            py::arg(), py::arg("same_config")=false)
        .def("toJointspace", py::overload_cast<bool, bool>(&ayk::CartesianPosition::toJointspace, py::const_))
        .def("toJointspace", py::overload_cast<>(&ayk::CartesianPosition::toJointspace, py::const_))

        .def("toCylindric", py::overload_cast<const ayk::JointPosition &, bool>(&ayk::CartesianPosition::toCylindric, py::const_), 
            py::arg(), py::arg("same_config")=false)
        .def("toCylindric", py::overload_cast<bool>(&ayk::CartesianPosition::toCylindric, py::const_))
        .def("toCylindric", py::overload_cast<>(&ayk::CartesianPosition::toCylindric, py::const_))

        .def("transformFromBaseToTCP", &ayk::CartesianPosition::transformFromBaseToTCP)
        .def("transformFromTCPToBase", &ayk::CartesianPosition::transformFromTCPToBase)
        .def_static("fromMsg", &ayk::CartesianPosition::fromMsg)
        .def("isReachable", &ayk::CartesianPosition::isReachable);

    // CartesianVelocity
    py::class_<ayk::CartesianVelocity, ayk::CartesianVector>(m, "CartesianVelocity")
        .def(py::init<>())
        .def(py::init<const std::vector<double>>())
        .def("toJointspace", &ayk::CartesianVelocity::toJointspace)
        .def("toCylindric", &ayk::CartesianVelocity::toCylindric)        
        .def("transformFromBaseToTCP", &ayk::CartesianVelocity::transformFromBaseToTCP)
        .def("transformFromTCPToBase", &ayk::CartesianVelocity::transformFromTCPToBase)    
        .def_static("fromMsg", &ayk::CartesianVelocity::fromMsg); 

    // CartesianAcceleration
    py::class_<ayk::CartesianAcceleration, ayk::CartesianVector>(m, "CartesianAcceleration")
        .def(py::init<>())
        .def(py::init<const std::vector<double>>());  
    

    // ---------- arm_dynamics.h ----------
    // JointEffort
    py::class_<ayk::JointEffort>(m, "JointEffort")
        .def(py::init<>())
        .def(py::init<const std::vector<double> &>())
        .def("toCylindric", &ayk::JointEffort::toCylindric)
        .def("toCartesian", &ayk::JointEffort::toCartesian);

    // CylindricEffort
    py::class_<ayk::CylindricEffort>(m, "CylindricEffort")
        .def(py::init<>())
        .def(py::init<const std::vector<double> &>())
        .def("toJointspace", &ayk::CylindricEffort::toJointspace)
        .def("toCartesian", &ayk::CylindricEffort::toCartesian);

    // CartesianEffort
    py::class_<ayk::CartesianEffort>(m, "CartesianEffort")
        .def(py::init<>())
        .def(py::init<const std::vector<double> &>())
        .def("toCylindric", &ayk::CartesianEffort::toCylindric)
        .def("toJointspace", &ayk::CartesianEffort::toJointspace);

    // YoubotArmDynamics
    py::class_<ayk::YoubotArmDynamics>(m, "YoubotArmDynamics")
        .def(py::init<>())
        .def("getStaticJointEffort", &ayk::YoubotArmDynamics::getStaticJointEffort)
        .def("setStaticParameters", &ayk::YoubotArmDynamics::setStaticParameters);

    // StaticParameters
    py::class_<ayk::StaticParameters>(m, "StaticParameters")
        .def(py::init<>())
        .def_readwrite("mass_5", &ayk::StaticParameters::mass_5)
        .def_readwrite("mass_4", &ayk::StaticParameters::mass_4)
        .def_readwrite("mass_3", &ayk::StaticParameters::mass_3)
        .def_readwrite("mass_2", &ayk::StaticParameters::mass_2)
        .def_readwrite("com_radius_5", &ayk::StaticParameters::com_radius_5)
        .def_readwrite("com_radius_4", &ayk::StaticParameters::com_radius_4)
        .def_readwrite("com_radius_3", &ayk::StaticParameters::com_radius_3)
        .def_readwrite("com_radius_2", &ayk::StaticParameters::com_radius_2)
        .def_readwrite("com_angle_5", &ayk::StaticParameters::com_angle_5)
        .def_readwrite("com_angle_4", &ayk::StaticParameters::com_angle_4)
        .def_readwrite("com_angle_3", &ayk::StaticParameters::com_angle_3)
        .def_readwrite("com_angle_2", &ayk::StaticParameters::com_angle_2)
        .def_readwrite("friction_5", &ayk::StaticParameters::friction_5)
        .def_readwrite("friction_4", &ayk::StaticParameters::friction_4)
        .def_readwrite("friction_3", &ayk::StaticParameters::friction_3)
        .def_readwrite("friction_2", &ayk::StaticParameters::friction_2)
        .def_readwrite("gravity", &ayk::StaticParameters::gravity);

}