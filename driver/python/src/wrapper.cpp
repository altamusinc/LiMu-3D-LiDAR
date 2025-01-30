#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <tof.hpp>

namespace py = pybind11;

PYBIND11_MODULE(limu_py, m) {
    pybind11::class_<ToF>(m, "ToF")
        .def("tof320", &ToF::tof320)
        .def("stopStream", &ToF::stopStream)
        .def("streamDCS", &ToF::streamDCS)
        .def("streamGrayscale", &ToF::streamGrayscale)
        .def("streamDistance", &ToF::streamDistance)
        .def("stremDistanceAmplitude", &ToF::streamDistanceAmplitude)
        .def("setOffset", &ToF::setOffset)
        .def("setMinAmplitude", &ToF::setMinAmplitude)
        .def("setBinning", &ToF::setBinning)
        .def("setRoi", &ToF::setRoi)
        .def("setIntegrationTime", &ToF::setIntegrationTime)
        .def("setHDRMode", &ToF::setHDRMode)
        .def("setModulation", &ToF::setModulation)
        .def("setFilter", &ToF::setFilter)
        .def("setPrecompute", &ToF::setPrecompute)
        .def("setIPAddress", &ToF::setIPAddress)
        .def("subscribeFrame", &ToF::subscribeFrame)
        .def("subscribeCameraInfo", &ToF::subscribeCameraInfo)
        .def("setLensType", &ToF::setLensType)
        .def("setLensCenter", &ToF::setLensCenter)
        .def("getWidth", &ToF::getWidth)
        .def("getHeight", &ToF::getHeight)
        ;
}