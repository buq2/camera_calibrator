#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "extrinsics_calibrator.hh"
#include "calibrator.hh"

namespace pybind11 { namespace detail {
    template <typename T> struct type_caster<Eigen::Transform<T,3,2,0>> {
    using Type = Eigen::Transform<T,3,2,0>;
    public:
        PYBIND11_TYPE_CASTER(Type, const_name("Eigen::Transform<T,3,2,0>"));

        /**
         * Conversion part 1 (Python->C++): convert a PyObject into a Eigen::Transform<float,3,2,0>
         * instance or return false upon failure. The second argument
         * indicates whether implicit conversions should be applied.
         */
        bool load(handle src, bool imp) {
            if (!src) {
              return false;
            }

            auto val = type_caster<Eigen::Matrix<T,4,4,0>>();
            if (!val.load(src, imp)) return false;
            value.linear() = (*val).block<3,3>(0,0);
            value.translation() = (*val).block<3,1>(0,3);
            return true;
        }

        /**
         * Conversion part 2 (C++ -> Python): convert an Eigen::Transform<T,3,2,0> instance into
         * a Python object. The second and third arguments are used to
         * indicate the return value policy and parent object (for
         * ``return_value_policy::reference_internal``) and are generally
         * ignored by implicit casters.
         */
        static handle cast(Type src, return_value_policy policy, handle parent) {
            auto val = type_caster<Eigen::Matrix<T,4,4>>();
            return val.cast(src.matrix().eval(), policy, parent);
        }
    };
}} // namespace pybind11::detail

PYBIND11_MODULE(pycalibrator, pycalibrator) {
  pycalibrator.doc() = "Camera calibrator";

  pybind11::class_<calibrator::Calibrator>(pycalibrator, "Calibrator")
        .def(pybind11::init<int, int>())
        .def("EstimateOpenCv", &calibrator::Calibrator::EstimateOpenCv, pybind11::arg("img_points"), pybind11::arg("world_points"))
        .def("Estimate", &calibrator::Calibrator::Estimate, pybind11::arg("img_points"), pybind11::arg("world_points"))
        .def("Optimize", &calibrator::Calibrator::Optimize)
        .def("GetK", &calibrator::Calibrator::GetK)
        .def("GetDistortion", &calibrator::Calibrator::GetDistortion)
        ;
  
  pybind11::class_<calibrator::ExtrinsicsCalibrator>(pycalibrator, "ExtrinsicsCalibrator")
      .def(pybind11::init<>())
      .def("AddCameraTRig", &calibrator::ExtrinsicsCalibrator::AddCameraTRig, pybind11::arg("camera_T_rig"), pybind11::arg("freeze") = false)
      .def("GetCameraTRig", &calibrator::ExtrinsicsCalibrator::GetCameraTRig, pybind11::arg("id"))
      .def("AddObservationFrame", &calibrator::ExtrinsicsCalibrator::AddObservationFrame, pybind11::arg("rig_T_world"))
      .def("GetObservationFrame", &calibrator::ExtrinsicsCalibrator::GetObservationFrame, pybind11::arg("id"))
      .def("AddWorldPoint", &calibrator::ExtrinsicsCalibrator::AddWorldPoint, pybind11::arg("frame_id"), pybind11::arg("world_point"))
      .def("AddObservation", &calibrator::ExtrinsicsCalibrator::AddObservation, pybind11::arg("camera_id"), pybind11::arg("world_point_id"), pybind11::arg("image_point"))
      .def("Optimize", &calibrator::ExtrinsicsCalibrator::Optimize)
      .def("Serialize", &calibrator::ExtrinsicsCalibrator::Serialize, pybind11::arg("fname"))
      .def("Parse", &calibrator::ExtrinsicsCalibrator::Parse, pybind11::arg("fname"))
      ;
};
