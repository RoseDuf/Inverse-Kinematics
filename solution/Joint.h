#ifndef JOINT_H
#define JOINT_H

#include "dependencies/Eigen/Dense"
#include "OpenGL/elements/joint2d.h"
using Eigen::Matrix3d;
using Eigen::MatrixXf;
using Eigen::Vector2d;

class Joint
{
public:
    Joint();
    Joint(Joint2D* correspondingJoint);
    Joint(const Joint& joint);
    ~Joint();
    friend bool operator==(const Joint& myJoint, const Joint& otherJoint)
    {
        return myJoint.GetJoint() == otherJoint.GetJoint();
    }

    inline float GetMagnitude() const {return m_Magnitude;}
    inline void SetMagnitude(float magnitude) {m_Magnitude = magnitude;}
    inline Matrix3d GetTransfMatrix() const {return m_TransfMatrix;}
    inline void SetTransfMatrix(Matrix3d transfMatrix) {m_TransfMatrix = transfMatrix;}
    inline Joint2D* GetJoint() const {return m_CorrespondingJoint;}
    inline void SetJoint(Joint2D* correspondingJoint) {m_CorrespondingJoint = correspondingJoint;}
    inline std::vector<Joint2D*> GetChildren() const {return m_Children;}
    inline void AddChild(Joint2D* child) {m_Children.push_back(child);}
    inline Vector2d GetPosition() const {return m_position;}
    inline void SetPosition(Vector2d position) {m_position = position;}

private:
    Vector2d m_position;
    float m_Magnitude;
    Matrix3d m_TransfMatrix;
    Joint2D* m_CorrespondingJoint;
    std::vector<Joint2D*> m_Children;
};

#endif // JOINT_H
