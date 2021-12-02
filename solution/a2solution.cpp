#include "a2solution.h"

#include "dependencies/Eigen/Dense"
#include "OpenGL/elements/link2d.h"
#include "OpenGL/elements/joint2d.h"
#include "OpenGL/elements/obstacle2d.h"

#include <QDebug>

#include <map>
#include <queue>
#include <algorithm>

A2Solution::A2Solution(std::vector<Joint2D*>& joints, std::vector<Link2D*>& links, std::vector<Obstacle2D*>& obstacles)
    :m_joints(joints), m_links(links), m_obstacles(obstacles)
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
            m_RotationalJoints.clear();
        }

        m_Selected = selected;
        m_MousePos = mouse_pos;

        applyTransformationsIK(mouse_pos);
        //applyTransformations(mouse_pos);
    }
}

void A2Solution::applyTransformationsIK(QVector2D mouse_pos)
{
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

     QVector2D goal = newMousePosition;

     //In this exercise, the only end-effector is m_Selected
     Joint * end_effector = new Joint(m_Selected);
     m_EffectorJoints.push_back(end_effector);

     //Allow flexibility in case there is more thand one effector.
     for (int i = 0; i < m_EffectorJoints.size(); i++)
     {
         effectorBranch(m_EffectorJoints[i]->GetJoint());
     }

     //double distanceToGoal = sqrt(distanceFromGoal(goal).x() * distanceFromGoal(goal).x() + distanceFromGoal(goal).y() * distanceFromGoal(goal).y());
     for (int iterations = 0; iterations < 1; iterations++)
     {
        //1. Find the jacobian
        MatrixXd jac = jacobian(m_RotationalJoints, m_EffectorJoints);

        //2. Find delta_theta
        VectorXd angle_changes_vector = dampedLeastSquares(jac, 15, distanceFromGoal(goal));

        //Apply new angle to each rotational joint and apply FK
        for (int i = 0; i < m_RotationalJoints.size(); i++)
        {
            Matrix3d worldTransf = Matrix3d::Identity();
            m_RotationMatrix = Matrix3d::Identity();
            m_TranslationTransf = Matrix3d::Identity();
            m_RotationTransf = Matrix3d::Identity();

            //3. theta = theta + delta_theta
            QVector2D pJoint;
            if (m_RotationalJoints[i]->GetJoint()->get_parents().size() > 0)
            {
                pJoint = m_RotationalJoints[i]->GetJoint()->get_parents()[0]->get_position();
            }
            else if (m_RotationalJoints[i]->GetJoint()->get_parents().size() == 0 || m_RotationalJoints[i]->GetJoint()->is_locked())
            {
                pJoint = m_RotationalJoints[i]->GetJoint()->get_position();
            }

            QVector2D newPosition = m_RotationalJoints[i]->GetJoint()->get_position();

            if (!m_RotationalJoints[i]->GetJoint()->is_locked())
            {
                Vector2d vectorToParent = convertToEigenMath(m_RotationalJoints[i]->GetJoint()->get_position() - pJoint);
                Vector3d rotationalJointVector = Vector3d(vectorToParent.x(), vectorToParent.y(), 0);
                //apply new rotation angle to joint and do Forward Kinematics
                m_RotationMatrix(0, 0) = cos(angle_changes_vector[i-1]);
                m_RotationMatrix(0, 1) = sin(angle_changes_vector[i-1]);
                m_RotationMatrix(1, 0) = -sin(angle_changes_vector[i-1]);
                m_RotationMatrix(1, 1) = cos(angle_changes_vector[i-1]);

                //Find small translation transformation for joint
                Vector3d newPos = m_RotationMatrix * rotationalJointVector;
                QVector2D rotatedPos = convertToQtMath(Vector2d(newPos.x(), newPos.y()));
                newPosition = pJoint + rotatedPos;
            }

            //4. FK transformations on each joint
            //might have to remove translations
            m_TranslationTransf = translationTransformation(m_RotationalJoints[i]->GetJoint(), newPosition);
            worldTransf *= m_TranslationTransf;
            Traverse(m_RotationalJoints[i]->GetJoint(), worldTransf);

            //5. Update new position from FK transformations
            for (int j = 0; j < m_Joints.size(); j++)
            {
                if (m_Joints[j]->GetJoint() == m_RotationalJoints[i]->GetJoint())
                {
                    parentJoint = m_Joints[j]->GetJoint();
                }
                if (parentJoint != nullptr)
                {
                    //solve matrices
                    Vector2d jointPosition = convertToEigenMath(m_Joints[j]->GetJoint()->get_position());
                    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> colPivHouseholderQr(m_Joints[j]->GetTransfMatrix());
                    Vector3d jointPosition3D = Vector3d(jointPosition.x(), jointPosition.y(), 1);
                    Vector3d newPosition3D = colPivHouseholderQr.solve(jointPosition3D);
                    Vector2d newPosition2D = Vector2d(newPosition3D.x(), newPosition3D.y());

                    //clamping
                    if (m_Joints[j]->GetJoint() != m_RotationalJoints[i]->GetJoint() && m_Joints[j]->GetJoint()->get_parents().size() > 0)
                    {
                        QVector2D newVecToJointParent = convertToQtMath(newPosition2D) - m_Joints[j]->GetJoint()->get_parents()[0]->get_position();
                        float angle = atan2(newVecToJointParent.y(), newVecToJointParent.x());
                        QVector2D newPosFromOrigin = QVector2D(m_Joints[j]->GetMagnitude() * cos(angle), m_Joints[j]->GetMagnitude() * sin(angle));
                        newPosition2D = convertToEigenMath(m_Joints[j]->GetJoint()->get_parents()[0]->get_position() + newPosFromOrigin);
                    }

                    //update positions
                    m_Joints[j]->GetJoint()->set_position(convertToQtMath(newPosition2D));

                    parentJoint = m_Joints[j]->GetJoint();
                }
            }

            m_Joints.clear();
        }

        //distanceToGoal = sqrt(distanceFromGoal(goal).x() * distanceFromGoal(goal).x() + distanceFromGoal(goal).y() * distanceFromGoal(goal).y());
     }

     m_EffectorJoints.clear();
}

Vector2d A2Solution::distanceFromGoal(QVector2D goal)
{
    Vector2d distanceVector = convertToEigenMath(m_Selected->get_position() - goal);
    return distanceVector;
}

void A2Solution::effectorBranch(Joint2D* sibling)
{
    if (sibling->get_parents().size() >= 0 && !sibling->is_locked())
    {
        int i = 0;
        while (i < sibling->get_parents().size())
        {
            if (!sibling->is_locked())
            {
                effectorBranch(sibling->get_parents()[i]);
            }

            i++;
        }
    }

    Joint * newJoint;
    bool createNewJoint = true;
    for (int i = 0; i < m_RotationalJoints.size(); i++)
    {
        if (sibling == m_RotationalJoints[i]->GetJoint())
        {
            createNewJoint = false;
            newJoint = m_RotationalJoints[i];
            break;
        }
    }
    if (createNewJoint)
    {
        newJoint = new Joint(sibling);
        m_RotationalJoints.push_back(newJoint);
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
            float angle = getRotationAngle(m_Selected, newMousePosition);
            m_RotationMatrix(0, 0) = cos(angle);
            m_RotationMatrix(0, 1) = -sin(angle);
            m_RotationMatrix(1, 0) = sin(angle);
            m_RotationMatrix(1, 1) = cos(angle);

            m_TranslationTransf = translationTransformation(m_Selected, newMousePosition);
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
        int i = 0;
        while (i < newJoint->GetChildren().size())
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

Matrix3d A2Solution::translationTransformation(Joint2D* joint, QVector2D mouse_pos)
{
    Vector2d translVector = convertToEigenMath(joint->get_position()) - convertToEigenMath(mouse_pos);
    Matrix3d translationMatrix = Matrix3d::Identity();
    translationMatrix.col(2) = Vector3d(translVector.x(), translVector.y(), 1);

    return translationMatrix;
}

float A2Solution::getRotationAngle(Joint2D* joint, QVector2D new_pos)
{
    QVector2D parentJoint = joint->get_position();
    if (joint->get_parents().size() > 0)
    {
        parentJoint = joint->get_parents()[0]->get_position();
    }
    Vector2d mouseVector = convertToEigenMath(new_pos - parentJoint);
    Vector2d jointVector = convertToEigenMath(joint->get_position() - parentJoint);
    float v1_angle = 0;
    float v2_angle = 0;
    if (mouseVector != Vector2d(0, 0) && jointVector != Vector2d(0, 0))
    {
        v1_angle = atan2(jointVector.y(), jointVector.x());
        v2_angle = atan2(mouseVector.y(), mouseVector.x());
    }

    return v1_angle - v2_angle;

}


void A2Solution::test_eigen_library(){

    // create a simple matrix 5 by 6
    MatrixXf mat(5,6);

    // Fills in matrix
    for(int row=0;row<mat.rows();row++){
        for(int col=0;col<mat.cols();col++){
            mat(row,col) = row+col;
        }
    }

    // create the pseudoinverse
    MatrixXf pseudo_inv = mat.completeOrthogonalDecomposition().pseudoInverse();

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

MatrixXd A2Solution::jacobian(std::vector<Joint*> joints, std::vector<Joint*> end_effectors)
{
    Vector3d axis_of_rotation = Vector3d(0, 0, 1);

    int num_columns = joints.size();
    int num_rows = 2 * end_effectors.size();
    MatrixXd jac(num_rows, num_columns);

    // Go through the columns of J
    for (int col = 0; col < num_columns; col++)
    {
        Vector3d joint_vectorForm(3);
        joint_vectorForm(0) = joints[col]->GetJoint()->get_position().x();
        joint_vectorForm(1) = joints[col]->GetJoint()->get_position().y();
        joint_vectorForm(2) = 0;

        std::vector<Vector3d> end_effectors_vector;
        for (int i = 0; i < end_effectors.size(); i++)
        {
            Vector3d end_effector_vectorForm(3);
            end_effector_vectorForm(0) = end_effectors[i]->GetJoint()->get_position().x();
            end_effector_vectorForm(1) = end_effectors[i]->GetJoint()->get_position().y();
            end_effector_vectorForm(2) = 0;

            end_effectors_vector.push_back(axis_of_rotation.cross(end_effector_vectorForm - joint_vectorForm));
        }

        for (int row = 0; row < num_rows; row += 2)
        {
            jac(row, col) = end_effectors_vector[row].x();
            jac(row + 1, col) = end_effectors_vector[row].y();
        }
    }

    return jac;
}

VectorXd A2Solution::dampedLeastSquares(MatrixXd jac, float damping_factor, VectorXd error_vector)
{
    MatrixXd transposeJac = jac.transpose();

    MatrixXd lambda_I(jac.rows(), jac.rows());
    lambda_I = lambda_I.setIdentity() * (damping_factor * damping_factor);

    MatrixXd changeInAngles = transposeJac * ((jac * transposeJac) + lambda_I).inverse();

    return changeInAngles * error_vector;
}
