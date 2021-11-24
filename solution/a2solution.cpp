#include "a2solution.h"

#include "dependencies/Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXf;

A2Solution::A2Solution(std::vector<Joint2D*>& joints, std::vector<Link2D*>& links)
    :m_joints(joints),
    m_links(links)
{
    m_Selected = nullptr;
    m_IsInitialized = false;
}

A2Solution::~A2Solution()
{
}

void A2Solution::initialize()
{
    for (int i = 0; i < m_joints.size(); i++)
    {
        m_CurrentJoints.push_back(m_joints[i]);
    }

    //m_OriginalPosition = convertToEigenMath(m_Selected->get_position());

    m_IsInitialized = true;
}

bool A2Solution::changesWereMade()
{
    if (!m_IsInitialized || m_CurrentJoints.size() != m_joints.size() || m_CurrentJoints != m_joints)
    {
        return true;
    }

    return false;
}

void A2Solution::update(Joint2D* selected, QVector2D mouse_pos)
{
    // Code to silence warnings (redundant once solution is implemented)
        (void)selected;
        (void)mouse_pos;

    if (changesWereMade())
    {
        m_IsInitialized = false;
    }

    if (!m_IsInitialized)
    {
        initialize();
    }

    if (m_IsInitialized)
    {
        if (m_Selected == nullptr || m_Selected->get_position() != selected->get_position())
        {
            m_Magnitude = 0;
            for (auto p : m_Joints)
            {
              delete p;
            }
            m_Joints.clear();
        }

        m_Selected = selected;
        m_MousePos = mouse_pos;

        applyTransformations(mouse_pos);
    }
}

void A2Solution::applyTransformations(QVector2D mouse_pos)
{
    Matrix3d worldTransf = Matrix3d::Identity();
    Joint2D* parentJoint = nullptr;

    if (m_Magnitude == 0)
    {
        QVector2D vecToJointParent = QVector2D(0, 0);
        if (m_Selected->get_parents().size() > 0)
        {
            vecToJointParent = m_Selected->get_position() - m_Selected->get_parents()[0]->get_position();
        }
        m_Magnitude = sqrt(vecToJointParent.x() * vecToJointParent.x() + vecToJointParent.y() * vecToJointParent.y());
    }

    //clamping
    QVector2D newMousePosition = QVector2D(0, 0);
    if (m_Selected->get_parents().size() > 0)
    {
        QVector2D vecToMouseParent = mouse_pos - m_Selected->get_parents()[0]->get_position();
        float angle = atan2(vecToMouseParent.y(), vecToMouseParent.x());
        QVector2D newPositionFromOrigin = QVector2D(m_Magnitude * cos(angle), m_Magnitude * sin(angle));
        newMousePosition = m_Selected->get_parents()[0]->get_position() + newPositionFromOrigin;
    }
    else
    {
        newMousePosition = mouse_pos;
    }

    m_RotationMatrix = Matrix3d::Identity();
    m_TranslationTransf = Matrix3d::Identity();
    m_RotationTransf = Matrix3d::Identity();

    for (int i = 0; i < m_CurrentJoints.size(); i++)
    {
        if (m_CurrentJoints[i] == m_Selected)
        {
            float angle = getRotationAngle(newMousePosition);
            m_RotationMatrix(0, 0) = cos(angle);
            m_RotationMatrix(0, 1) = -sin(angle);
            m_RotationMatrix(1, 0) = sin(angle);
            m_RotationMatrix(1, 1) = cos(angle);

            m_TranslationTransf = translationTransformation(newMousePosition);
            worldTransf *= m_TranslationTransf;

            Traverse(m_CurrentJoints[i], worldTransf);
            break;
        }
    }

    for (int i = 0; i < m_Joints.size(); i++)
    {
        if (m_Joints[i]->GetJoint() == m_Selected)
        {
            parentJoint = m_Joints[i]->GetJoint();
        }
        if (parentJoint != nullptr)
        {
            //solve matrices
            Vector2d jointPosition = convertToEigenMath(m_Joints[i]->GetJoint()->get_position());
            Eigen::ColPivHouseholderQR<Eigen::MatrixXd> colPivHouseholderQr(m_Joints[i]->GetTransfMatrix());
            Vector3d jointPosition3D = Vector3d(jointPosition.x(), jointPosition.y(), 1);
            Vector3d newPosition3D = colPivHouseholderQr.solve(jointPosition3D);
            Vector2d newPosition2D = Vector2d(newPosition3D.x(), newPosition3D.y());


            if (m_Joints[i]->GetJoint() != m_Selected && m_Joints[i]->GetJoint()->get_parents().size() > 0)
            {
                QVector2D newVecToJointParent = convertToQtMath(newPosition2D) - m_Joints[i]->GetJoint()->get_parents()[0]->get_position();
                float angle = atan2(newVecToJointParent.y(), newVecToJointParent.x());
                QVector2D newPosFromOrigin = QVector2D(m_Joints[i]->GetMagnitude() * cos(angle), m_Joints[i]->GetMagnitude() * sin(angle));
                newPosition2D = convertToEigenMath(m_Joints[i]->GetJoint()->get_parents()[0]->get_position() + newPosFromOrigin);
            }

            m_Joints[i]->GetJoint()->set_position(convertToQtMath(newPosition2D));

            parentJoint = m_Joints[i]->GetJoint();
        }
    }
}


void A2Solution::Traverse(Joint2D* sibling, Matrix3d worldTransf)
{
    Joint * newJoint;
    bool createNewJoint = true;
    for (int i = 0; i < m_Joints.size(); i++)
    {
        if (sibling == m_Joints[i]->GetJoint())
        {
            createNewJoint = false;
            newJoint = m_Joints[i];
            break;
        }
    }
    if (createNewJoint)
    {
        newJoint = new Joint(sibling);
    }

    //transformation matrices
    m_RotationTransf = rotationTransformation(m_RotationMatrix, newJoint->GetJoint());
    worldTransf *= m_RotationTransf;

    newJoint->SetTransfMatrix(worldTransf);

    if (createNewJoint)
    {
        m_Joints.push_back(newJoint);
    }

    if (newJoint->GetChildren().size() > 0)
    {
        int nbrOfChildren = newJoint->GetChildren().size();
        int i = 0;
        while (i < nbrOfChildren)
        {
            Traverse(newJoint->GetChildren()[i], newJoint->GetTransfMatrix());
            i++;
        }
    }

}

Matrix3d A2Solution::rotationTransformation(Matrix3d rotationMatrix, Joint2D* joint)
{
    Vector2d jointPosition = convertToEigenMath(joint->get_position());

    Matrix3d firstTranslation = Matrix3d::Identity();
    Vector2d translation = -Vector2d(jointPosition);
    firstTranslation.col(2) = Vector3d(translation.x(), translation.y(), 1);

    Matrix3d secondTranslation = Matrix3d::Identity();
    translation = Vector2d(jointPosition);
    secondTranslation.col(2) = Vector3d(translation.x(), translation.y(), 1);

    Matrix3d rotationTransf = secondTranslation * rotationMatrix * firstTranslation;

    return rotationTransf;
}

Matrix3d A2Solution::translationTransformation(QVector2D mouse_pos)
{
    Vector2d translVector = convertToEigenMath(m_Selected->get_position()) - convertToEigenMath(mouse_pos);
    Matrix3d translationMatrix = Matrix3d::Identity();
    translationMatrix.col(2) = Vector3d(translVector.x(), translVector.y(), 1);

    return translationMatrix;
}

float A2Solution::getRotationAngle(QVector2D mouse_pos)
{
    QVector2D parentJoint = m_Selected->get_position();
    if (m_Selected->get_parents().size() > 0)
    {
        parentJoint = m_Selected->get_parents()[0]->get_position();
    }
    Vector2d mouseVector = convertToEigenMath(mouse_pos - parentJoint);
    Vector2d jointVector = convertToEigenMath(m_Selected->get_position() - parentJoint);
    float v1_angle = 0;
    float v2_angle = 0;
    if (mouseVector != Vector2d(0, 0) && jointVector != Vector2d(0, 0))
    {
        v1_angle = atan2(jointVector.y(), jointVector.x());
        v2_angle = atan2(mouseVector.y(), mouseVector.x());
    }

    return v1_angle - v2_angle;

}


void A2Solution::test_eigen_library()
{

    // create a simple matrix 5 by 6
    MatrixXd mat(5,6);

    // Fills in matrix
    for(unsigned int row=0;row<mat.rows();row++){
        for(unsigned int col=0;col<mat.cols();col++){
            mat(row,col) = row+col;
        }
    }

    // create the pseudoinverse
    MatrixXd pseudo_inv = mat.completeOrthogonalDecomposition().pseudoInverse();

    // print the pseudoinverse
    std::cout << "--------------------------" << std::endl;
    std::cout << pseudo_inv << std::endl;
    std::cout << "--------------------------" << std::endl;

}

Vector2d A2Solution::convertToEigenMath(QVector2D vector)
{
    return Vector2d(vector.x(), -vector.y());
}

QVector2D A2Solution::convertToQtMath(Vector2d vector)
{
    return QVector2D(vector.x(), -vector.y());
}
