// Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
#pragma once
#if defined(_MSC_VER)
#pragma warning(disable : 4251)
#pragma warning(disable : 4267)
#pragma warning(disable : 4554)
#pragma warning(disable : 4996)
#endif
#include <memory>
#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>
#include <pybind11/eigen/tensor.h>
namespace py=pybind11;
namespace nl=nlohmann;
//Ruler,Reward
PYBIND11_MAKE_OPAQUE(std::vector<std::string>);
PYBIND11_MAKE_OPAQUE(std::map<std::string,std::string>);
PYBIND11_MAKE_OPAQUE(std::map<std::string,int>);
PYBIND11_MAKE_OPAQUE(std::map<std::string,double>);
PYBIND11_MAKE_OPAQUE(std::map<std::string,Eigen::Vector2d>);
//Agent
class PhysicalAssetAccessor;
PYBIND11_MAKE_OPAQUE(std::map<std::string,std::shared_ptr<PhysicalAssetAccessor>>);
//Asset
class CommunicationBuffer;
PYBIND11_MAKE_OPAQUE(std::map<std::string,std::weak_ptr<CommunicationBuffer>>);
//PhysicalAsset
PYBIND11_MAKE_OPAQUE(std::map<std::string,bool>);
//Fighter
class Missile;
PYBIND11_MAKE_OPAQUE(std::vector<std::weak_ptr<Missile>>);
class Track3D;
PYBIND11_MAKE_OPAQUE(std::vector<std::pair<Track3D,bool>>);
PYBIND11_MAKE_OPAQUE(std::vector<Track3D>);
class Track2D;
PYBIND11_MAKE_OPAQUE(std::vector<std::pair<Track2D,bool>>);
PYBIND11_MAKE_OPAQUE(std::vector<Track2D>);
PYBIND11_MAKE_OPAQUE(std::vector<std::vector<std::string>>);
PYBIND11_MAKE_OPAQUE(std::deque<std::pair<double,nl::json>>);
//Missile
PYBIND11_MAKE_OPAQUE(std::vector<Eigen::VectorXd>);
//Sensor
PYBIND11_MAKE_OPAQUE(std::vector<Eigen::VectorXi>);
//StaticCollisionAvoider2D
class BorderGeometry;
PYBIND11_MAKE_OPAQUE(std::vector<std::shared_ptr<BorderGeometry>>);

#define __TOSTRING(...) #__VA_ARGS__
#define BIND_VECTOR_NAME(value_type,name,local) py::bind_vector<std::vector<value_type>>(m,name,py::module_local(local))\
    .def("tolist",\
        [](std::vector<value_type> &v)->py::list{\
            py::list ret=py::reinterpret_steal<py::list>(py::detail::list_caster<std::vector<value_type>,value_type>::cast(v,py::return_value_policy::copy,py::none()));\
            return ret;\
        }\
    )\
    .def(py::pickle(\
        [](std::vector<value_type> &v)->py::list{\
            py::list ret=py::reinterpret_steal<py::list>(py::detail::list_caster<std::vector<value_type>,value_type>::cast(v,py::return_value_policy::copy,py::none()));\
            return ret;\
        },\
        [](py::list t)->std::vector<value_type>{\
            py::detail::list_caster<std::vector<value_type>,value_type> caster;\
            caster.load(t,true);\
            std::vector<value_type> ret=caster;\
            return ret;\
        }\
    ))
#define BIND_VECTOR(value_type,local) BIND_VECTOR_NAME(value_type,__TOSTRING(std::vector<value_type>),local)
#define BIND_MAP_NAME(key_type,value_type,name,local) py::bind_map<std::map<key_type,value_type>>(m,name,py::module_local(local))\
    .def("keys",\
           [](std::map<key_type,value_type> &m) { return py::make_key_iterator(m.begin(), m.end()); },\
           py::keep_alive<0, 1>()\
    )\
    .def("values",\
           [](std::map<key_type,value_type> &m) { return py::make_value_iterator(m.begin(), m.end()); },\
           py::keep_alive<0, 1>()\
    )\
    .def("todict",\
        [](std::map<key_type,value_type> &m)->py::dict{\
            py::dict ret=py::reinterpret_steal<py::dict>(py::detail::map_caster<std::map<key_type,value_type>,key_type,value_type>::cast(m,py::return_value_policy::copy,py::none()));\
            return ret;\
        }\
    )\
    .def(py::pickle(\
        [](std::map<key_type,value_type> &m)->py::dict{\
            py::dict ret=py::reinterpret_steal<py::dict>(py::detail::map_caster<std::map<key_type,value_type>,key_type,value_type>::cast(m,py::return_value_policy::copy,py::none()));\
            return ret;\
        },\
        [](py::dict t)->std::map<key_type,value_type>{\
            py::detail::map_caster<std::map<key_type,value_type>,key_type,value_type> caster;\
            caster.load(t,true);\
            std::map<key_type,value_type> ret=caster;\
            return ret;\
        }\
    ))
#define BIND_MAP(key_type,value_type,local) BIND_MAP_NAME(key_type,value_type,__TOSTRING(std::map<key_type,value_type>),local)
namespace util{
    template<typename KeyType,typename ValueType>
    py::dict todict(const std::map<KeyType,ValueType>& m){
        return py::reinterpret_steal<py::dict>(py::detail::map_caster<std::map<KeyType,ValueType>,KeyType,ValueType>::cast(m,py::return_value_policy::copy,py::none()));
    }
    template<typename ValueType>
    py::list tolist(const std::vector<ValueType>& v){
        return py::reinterpret_steal<py::list>(py::detail::list_caster<std::vector<ValueType>,ValueType>::cast(v,py::return_value_policy::copy,py::none()));
    }
};

void PYBIND11_EXPORT exportCommon(py::module &m);