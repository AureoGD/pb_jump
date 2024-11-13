#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "jump_rgc/op_wrapper.h"
#include "jump_rgc/jump_robot_model.h"

namespace py = pybind11;

PYBIND11_MODULE(pybind_opWrapper, m)
{
    py::class_<JumpRobot>(m, "JumpRobot")
        .def(py::init<>())
        .def("UpdateSysMatrices", &JumpRobot::UpdateSysMatrices)
        .def_readwrite("com_pos_w", &JumpRobot::com_pos_w)
        .def_readwrite("com_vel", &JumpRobot::com_vel)
        .def_readwrite("foot_pos", &JumpRobot::foot_pos)
        .def_readwrite("foot_vel", &JumpRobot::foot_vel);

    py::class_<Op_Wrapper>(m, "Op_Wrapper")
        .def(py::init<>())
        .def("RGCConfig", &Op_Wrapper::RGCConfig)
        .def("UpdateSt", &Op_Wrapper::UpdateSt)
        .def("ChooseRGCPO", &Op_Wrapper::ChooseRGCPO)
        .def("ResetPO", &Op_Wrapper::ResetPO)
        .def_readwrite("com_pos_w", &Op_Wrapper::com_pos_w)
        .def_readwrite("com_vel", &Op_Wrapper::com_vel)
        .def_readwrite("foot_pos", &Op_Wrapper::foot_pos)
        .def_readwrite("foot_vel", &Op_Wrapper::foot_vel)
        .def_readwrite("delta_qr", &Op_Wrapper::delta_qr);
}
