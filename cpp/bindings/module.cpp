#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "awake_slam/slam_core.hpp"

namespace py = pybind11;
using awake_slam::Pose2;
using awake_slam::Scan;
using awake_slam::SlamCore;
using awake_slam::SlamResult;

PYBIND11_MODULE(_awake_slam, m) {
    m.doc() = "AWAKE 2027 C++ SLAM core (M0 skeleton; full pipeline in M3)";

    py::class_<Pose2>(m, "Pose2")
        .def(py::init<>())
        .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"), py::arg("theta"))
        .def_readwrite("x", &Pose2::x)
        .def_readwrite("y", &Pose2::y)
        .def_readwrite("theta", &Pose2::theta)
        .def("__repr__", [](const Pose2& p) {
            return "Pose2(x=" + std::to_string(p.x) + ", y=" + std::to_string(p.y) +
                   ", theta=" + std::to_string(p.theta) + ")";
        });

    py::class_<Scan>(m, "Scan")
        .def(py::init<>())
        .def(py::init<std::vector<float>, std::vector<float>, double>(),
             py::arg("ranges"), py::arg("angles"), py::arg("t_start") = 0.0)
        .def_readwrite("ranges", &Scan::ranges)
        .def_readwrite("angles", &Scan::angles)
        .def_readwrite("t_start", &Scan::t_start)
        .def("size", &Scan::size)
        .def("valid", &Scan::valid);

    py::class_<SlamResult>(m, "SlamResult")
        .def_readonly("pose", &SlamResult::pose)
        .def_readonly("initialized", &SlamResult::initialized)
        .def_readonly("scan_count", &SlamResult::scan_count);

    py::class_<SlamCore>(m, "SlamCore")
        .def(py::init<double>(), py::arg("map_resolution") = 0.05)
        .def("process", &SlamCore::process, py::arg("scan"), py::arg("odometry"))
        .def("reset", &SlamCore::reset)
        .def_property_readonly("pose", &SlamCore::pose)
        .def_property_readonly("scan_count", &SlamCore::scan_count)
        .def_property_readonly("map_resolution", &SlamCore::map_resolution);
}
