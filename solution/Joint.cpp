#include "Joint.h"

Joint::Joint()
{
    m_TransfMatrix = Matrix3d::Identity();
    m_CorrespondingJoint = NULL;
    m_Magnitude = 0;
}

Joint::Joint(Joint2D* correspondingJoint)
{
    m_TransfMatrix = Matrix3d::Identity();
    m_CorrespondingJoint = correspondingJoint;
    m_Magnitude = 0;

    for (int i = 0; i < correspondingJoint->get_children().size(); i++)
    {
        m_Children.push_back(correspondingJoint->get_children()[i]);
    }

    if (m_CorrespondingJoint->get_parents().size() > 0)
    {
        QVector2D vecJointToParent = m_CorrespondingJoint->get_position() - m_CorrespondingJoint->get_parents()[0]->get_position();
        m_Magnitude = sqrt(vecJointToParent.x() * vecJointToParent.x() + vecJointToParent.y() * vecJointToParent.y());
    }
}

Joint::Joint(const Joint& joint)
{
    m_TransfMatrix = joint.GetTransfMatrix();
    m_CorrespondingJoint = joint.GetJoint();
}

Joint::~Joint()
{
}
