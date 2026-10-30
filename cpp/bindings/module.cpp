#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "awake_slam/slam_core.hpp"

namespace py = pybind11;
using awake_slam::Pose2;
using awake_slam::Scan;
using awake_slam::SlamCore;
using awake_slam::SlamResult;

PYBIND11_MODULE(_awake_slam, m) {
    m.doc() = "AWAKE 2027 2D lidar SLAM core (occupancy grid + correlative matching)";

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
        .def_readonly("scan_count", &SlamResult::scan_count)
        .def_readonly("match_score", &SlamResult::match_score)
        .def_readonly("inliers", &SlamResult::inliers)
        .def_readonly("loop_closures", &SlamResult::loop_closures)
        .def_readonly("loop_closure_this_scan", &SlamResult::loop_closure_this_scan)
        .def_readonly("graph_nodes", &SlamResult::graph_nodes)
        .def_readonly("graph_edges", &SlamResult::graph_edges)
        .def_readonly("loop_candidate", &SlamResult::loop_candidate)
        .def_readonly("loop_match_x", &SlamResult::loop_match_x)
        .def_readonly("loop_match_y", &SlamResult::loop_match_y)
        .def_readonly("loop_match_theta", &SlamResult::loop_match_theta)
        .def_readonly("mcl_pose", &SlamResult::mcl_pose)
        .def_readonly("mcl_initialized", &SlamResult::mcl_initialized)
        .def_readonly("mcl_ess", &SlamResult::mcl_ess)
        .def_readonly("mcl_spread", &SlamResult::mcl_spread)
        .def_readonly("mcl_particle_count", &SlamResult::mcl_particle_count)
        .def_readonly("global_relocalizations", &SlamResult::global_relocalizations)
        .def_readonly("relocalized_this_scan", &SlamResult::relocalized_this_scan);

    py::class_<SlamCore>(m, "SlamCore")
        .def(py::init<double, int, int, double, double, double, double, double>(),
             py::arg("map_resolution") = 0.05,
             py::arg("width_cells") = 300,
             py::arg("height_cells") = 160,
             py::arg("origin_x") = 0.0,
             py::arg("origin_y") = 0.0,
             py::arg("lidar_offset_x") = 0.10,
             py::arg("lidar_offset_y") = 0.0,
             py::arg("max_range") = 12.0)
        .def("process", &SlamCore::process, py::arg("scan"), py::arg("odometry"))
        .def("reset", &SlamCore::reset)
        .def("set_mcl_enabled", &SlamCore::set_mcl_enabled, py::arg("enabled"))
        .def("set_loop_closure_enabled", &SlamCore::set_loop_closure_enabled, py::arg("enabled"))
        .def("set_global_relocalization_enabled", &SlamCore::set_global_relocalization_enabled, py::arg("enabled"))
        .def("request_global_relocalization", &SlamCore::request_global_relocalization)
        .def("set_map_frozen", &SlamCore::set_map_frozen, py::arg("frozen"))
        .def_property_readonly("mcl_enabled", &SlamCore::mcl_enabled)
        .def_property_readonly("loop_closure_enabled", &SlamCore::loop_closure_enabled)
        .def_property_readonly("global_relocalization_enabled", &SlamCore::global_relocalization_enabled)
        .def_property_readonly("map_frozen", &SlamCore::map_frozen)
        .def("mcl_particles", &SlamCore::mcl_particles)
        .def("map_log_odds", &SlamCore::map_log_odds)
        .def_property_readonly("pose", &SlamCore::pose)
        .def_property_readonly("scan_count", &SlamCore::scan_count)
        .def_property_readonly("map_resolution", &SlamCore::map_resolution)
        .def_property_readonly("map_width", &SlamCore::map_width)
        .def_property_readonly("map_height", &SlamCore::map_height)
        .def_property_readonly("map_origin_x", &SlamCore::map_origin_x)
        .def_property_readonly("map_origin_y", &SlamCore::map_origin_y);
}
