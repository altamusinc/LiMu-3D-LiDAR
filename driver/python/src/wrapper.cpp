#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <tof.hpp>
#include <cstdio>

namespace py = pybind11;

PYBIND11_MODULE(limu_py, m) {
    py::class_<ToF>(m, "ToF")
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
    
    py::class_<Frame>(m, "Frame")
        .def_readwrite("stride", &Frame::stride)
        .def_readwrite("dataType", &Frame::dataType)
        .def_readwrite("width", &Frame::width)
        .def_readwrite("height", &Frame::height)
        .def_readwrite("payloadHeaderOffset", &Frame::payloadHeaderOffset)
        .def_readwrite("px_size;", &Frame::px_size)
        .def_readwrite("frame_id", &Frame::frame_id)
        .def_readwrite("distData", &Frame::distData)
        .def_readwrite("amplData", &Frame::amplData)
        .def_readwrite("dcsData", &Frame::dcsData)
        .def_readwrite("n_points", &Frame::n_points)
        .def_readwrite("data_depth_ptr", &Frame::data_depth)
        .def_readwrite("data_grayscale_ptr", &Frame::data_grayscale)
        .def_readwrite("data_amplitude_ptr", &Frame::data_amplitude)
        .def_readwrite("data_2d_bgr_ptr", &Frame::data_2d_bgr)
        .def_readwrite("saturated_mask_ptr", &Frame::saturated_mask)
        .def("get_xyz_rgb", [](Frame &frame) {
            auto vecsize = frame.n_points * 8;
            std::vector<float> vec {frame.data_3d_xyz_rgb, frame.data_3d_xyz_rgb + vecsize};
            return vec;
        })
        ;
}