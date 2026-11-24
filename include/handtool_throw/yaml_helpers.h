// Pose and scalar readers of the yaml parameters, shared by the testing node and the
// gazebo plugin: the object list must be read the same way by both.
#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <xmlrpcpp/XmlRpcValue.h>

namespace handtool_throw
{

// --- reads a scalar: yaml writes 3 as an int and 3.0 as a double, accept both --- //
inline bool getDouble(XmlRpc::XmlRpcValue &val, double &out)
{
	if (val.getType() == XmlRpc::XmlRpcValue::TypeDouble){
		out = static_cast<double>(val);
		return true;
	}
	if (val.getType() == XmlRpc::XmlRpcValue::TypeInt){
		out = static_cast<int>(val);
		return true;
	}
	return false;
}

// --- reads a [x, y, z] list --- //
inline bool getVector3(XmlRpc::XmlRpcValue &val, Eigen::Vector3d &out)
{
	if ((val.getType() != XmlRpc::XmlRpcValue::TypeArray) || (val.size() != 3)){
		return false;
	}
	for (int i = 0; i < 3; i++){
		double value;
		if (!getDouble(val[i], value)){
			return false;
		}
		out(i) = value;
	}
	return true;
}

// --- reads an orientation, given as a quaternion or as rpy angles --- //
inline bool getQuaternion(XmlRpc::XmlRpcValue &val, Eigen::Quaterniond &quat)
{
	if (val.getType() != XmlRpc::XmlRpcValue::TypeStruct){
		return false;
	}
	if (val.hasMember("orientation")){
		// - quaternion [x, y, z, w], the form the throwing service answers with - //
		XmlRpc::XmlRpcValue &quat_par = val["orientation"];
		if ((quat_par.getType() != XmlRpc::XmlRpcValue::TypeArray) || (quat_par.size() != 4)){
			return false;
		}
		double qx, qy, qz, qw;
		if ((!getDouble(quat_par[0], qx)) || (!getDouble(quat_par[1], qy))
			|| (!getDouble(quat_par[2], qz)) || (!getDouble(quat_par[3], qw))){
			return false;
		}
		quat = Eigen::Quaterniond(qw, qx, qy, qz);
	}else if (val.hasMember("rpy_deg")){
		// - extrinsic xyz angles, same convention as dummy_qualisys.py, easier to write by hand - //
		Eigen::Vector3d rpy;
		if (!getVector3(val["rpy_deg"], rpy)){
			return false;
		}
		rpy *= M_PI/180.0;
		quat = Eigen::Quaterniond(
			Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ())
			* Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()));
	}else{
		return false;
	}
	// a null quaternion cannot be normalised into a rotation
	if (quat.norm() < 1e-9){
		return false;
	}
	quat.normalize();
	return true;
}

// --- reads a pose: a 'position' list plus an orientation --- //
inline bool getPose(XmlRpc::XmlRpcValue &val, Eigen::Affine3d &out)
{
	Eigen::Vector3d position;
	Eigen::Quaterniond quat;
	if (val.getType() != XmlRpc::XmlRpcValue::TypeStruct){
		return false;
	}
	if ((!val.hasMember("position")) || (!getVector3(val["position"], position))){
		return false;
	}
	if (!getQuaternion(val, quat)){
		return false;
	}

	// a default constructed Affine3d is uninitialized, the last row would stay garbage
	out.setIdentity();
	out.translation() = position;
	out.linear() = quat.toRotationMatrix();
	return true;
}

}  // namespace handtool_throw
