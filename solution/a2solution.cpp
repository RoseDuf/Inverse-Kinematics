
#include "a2solution.h"
#include <QDebug>
#include <map>
#include <algorithm>

A2Solution::A2Solution(std::vector<Joint2D*>& joints, std::vector<Link2D*>& links, std::vector<Obstacle2D*>& obstacles)
    :m_joints(joints), m_links(links), m_obstacles(obstacles)
{
    m_Selected = nullptr;
    m_IsInitialized = false;
}

A2Solution::~A2Solution()
{
    clearData();
}

void A2Solution::initialize()
{
    for (int i = 0; i < m_joints.size(); i++)
    {
        m_CurrentJoints.push_back(m_joints[i]);
    }

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
        //If you deselect the joint, reset all data
        if (m_Selected == nullptr || m_Selected->get_position() != selected->get_position())
        {
            clearData();
        }

        m_Selected = selected;

        //Translate at the root
        if (m_Selected->get_parents().empty())
        {
            m_TranslationMatrix = Matrix3d::Identity();
            Vector2d translation_vec = convertToEigenMath(mouse_pos - selected->get_position());
            m_TranslationMatrix.col(2) = Vector3d(translation_vec.x(), translation_vec.y(), 1);
            TranslationTransformations(m_Selected, translation_vec);
            updatePositions();
        }

        applyTransformationsIK(mouse_pos);

        //6. Update all positions
        for (int i = 0; i < m_RotationalJoints.size(); i++)
        {
            m_RotationalJoints[i]->GetJoint()->set_position(convertToQtMath(m_RotationalJoints[i]->GetPosition()));
        }
    }
}

void A2Solution::applyTransformationsIK(QVector2D newMousePosition)
{
     if (m_RotationalJoints.size() == 0)
     {
         constructTreeFromRoot(m_Selected);
     }

     if (m_EffectorJoints.size() == 0)
     {
         Joint* e_Joint = new Joint(m_Selected);
         e_Joint->SetPosition(convertToEigenMath(m_Selected->get_position()));
         e_Joint->SetIsEndEffector(true);
         m_EffectorJoints.push_back(e_Joint);
     }

     for (int iterations = 0; iterations < 50; iterations++)
     {
        //1. Find the jacobian
        MatrixXd jac = jacobian(m_EffectorJoints);

        //2. Find delta_theta
        VectorXd angle_changes_vector = dampedLeastSquares(jac, 12, distanceFromGoal(newMousePosition, m_EffectorJoints));

        //3. Apply new angle to each rotational joint and apply FK
        for (int i = 1; i < m_RotationalJoints.size(); i++)
        {
            if (m_RotationalJoints[i]->GetJoint()->get_parents().size() == 0 || m_RotationalJoints[i]->GetJoint()->is_locked())
            {
                continue;
            }

            Vector2d pJoint;
            for (int p = 0; p < m_RotationalJoints.size(); p++)
            {
                if (m_RotationalJoints[i]->GetJoint()->get_parents()[0] == m_RotationalJoints[p]->GetJoint())
                {
                    pJoint = m_RotationalJoints[p]->GetPosition();
                    break;
                }
            }

            double angle = 0;

            if (is_descendant_of(m_EffectorJoints[0]->GetJoint(), m_RotationalJoints[i]->GetJoint()) || m_RotationalJoints[i]->GetIsEndEffector())
                angle = angle_changes_vector[i-1];

            m_RotationMatrix = Matrix3d::Identity();
            m_RotationMatrix(0, 0) = cos(angle);
            m_RotationMatrix(0, 1) = sin(angle);
            m_RotationMatrix(1, 0) = -sin(angle);
            m_RotationMatrix(1, 1) = cos(angle);

            Vector2d parent_to_joint = m_RotationalJoints[i]->GetPosition() - pJoint;

            //4. FK
            RotationTransformations(m_RotationalJoints[i]->GetJoint(), parent_to_joint, pJoint);
        }
     }
}

void A2Solution::RotationTransformations(Joint2D* joint, Vector2d parent_to_joint, Vector2d parent_pos)
{
    Joint * newJoint = nullptr;
    for (int i = 0; i < m_Joints.size(); i++)
    {
        if (joint == m_Joints[i]->GetJoint())
        {
            newJoint = m_Joints[i];
            break;
        }
    }

    Vector2d old_pos;
    for (int i = 0; i < m_RotationalJoints.size(); i++)
    {
        if (joint == m_RotationalJoints[i]->GetJoint())
        {
            old_pos = m_RotationalJoints[i]->GetPosition();
        }
    }

    Vector3d new_pos = (m_RotationMatrix * Vector3d(parent_to_joint.x(), parent_to_joint.y(), 1)) + Vector3d(parent_pos.x(), parent_pos.y(), 1);
    Vector2d new_pos_2d = Vector2d(new_pos.x(), new_pos.y());

    if (newJoint == nullptr)
    {
        newJoint = new Joint(joint);
        newJoint->SetPosition(new_pos_2d);
        m_Joints.push_back(newJoint);
    }

    if (newJoint->GetChildren().size() > 0)
    {
        int i = 0;
        while (i < newJoint->GetChildren().size())
        {
            Joint* child = nullptr;

            for (int r = 0; r < m_RotationalJoints.size(); r++)
            {
                if (newJoint->GetChildren()[i] == m_RotationalJoints[r]->GetJoint())
                {
                    child = m_RotationalJoints[r];
                }
            }

            if (child != nullptr)
            {
                Vector2d pJoint = child->GetPosition() - old_pos;
                RotationTransformations(newJoint->GetChildren()[i], pJoint, newJoint->GetPosition());
            }

            i++;
        }
    }

    //5. Update new position from FK transformations
    updatePositions();
}

void A2Solution::TranslationTransformations(Joint2D* joint, Vector2d translation_vec)
{
    Joint * newJoint = nullptr;
    for (int i = 0; i < m_Joints.size(); i++)
    {
        if (joint == m_Joints[i]->GetJoint())
        {
            newJoint = m_Joints[i];
            break;
        }
    }

    Vector2d pos2D = convertToEigenMath(joint->get_position());
    Vector3d new_pos = m_TranslationMatrix * Vector3d(pos2D.x(), pos2D.y(), 1);
    Vector2d new_pos_2d = Vector2d(new_pos.x(), new_pos.y());

    Vector2d parent_pos(0, 0);
    if (!joint->get_parents().empty())
        parent_pos = convertToEigenMath(joint->get_parents()[0]->get_position());

    if (newJoint == nullptr)
    {
        newJoint = new Joint(joint);
        newJoint->SetPosition(new_pos_2d);
        m_Joints.push_back(newJoint);
    }

    if (newJoint->GetChildren().size() > 0)
    {
        int i = 0;
        while (i < newJoint->GetChildren().size())
        {
            TranslationTransformations(newJoint->GetChildren()[i], translation_vec);
            i++;
        }
    }

    //5. Update new position from FK transformations
    updatePositions();
}

void A2Solution::updatePositions()
{
    for (int j = 0; j < m_Joints.size(); j++)
    {
        for (int x = 0; x < m_RotationalJoints.size(); x++)
        {
            if (m_Joints[j]->GetJoint() == m_RotationalJoints[x]->GetJoint())
            {
                m_RotationalJoints[x]->SetPosition(m_Joints[j]->GetPosition());
                if (m_RotationalJoints[x]->GetIsEndEffector())
                {
                    m_EffectorJoints[0]->SetPosition(m_Joints[j]->GetPosition());
                }
                break;
            }
        }
    }

    for (auto p : m_Joints)
    {
      delete p;
    }
    m_Joints.clear();
}

Vector2d A2Solution::distanceFromGoal(QVector2D goal, std::vector<Joint*> end_effectors)
{
    Vector2d distanceVector = Vector2d(0, 0);

    for (int i = 0; i < end_effectors.size(); i++)
    {
        if (end_effectors[i]->GetJoint() == m_Selected)
        {
            distanceVector = convertToEigenMath(goal) - end_effectors[i]->GetPosition();
            // clamp/downscale error vector
            if (distanceVector.norm() != 0)
                distanceVector *= (std::min(1.0, distanceVector.norm()) / distanceVector.norm());
            break;
        }
    }

    return distanceVector;
}

void A2Solution::effectorBranch(Joint2D* sibling)
{
    Joint * newJoint = nullptr;
    for (int i = 0; i < m_Joints.size(); i++)
    {
        if (sibling == m_Joints[i]->GetJoint())
        {
            newJoint = m_Joints[i];
            break;
        }
    }

    if (newJoint == nullptr)
    {
        newJoint = new Joint(sibling);
        newJoint->SetPosition(convertToEigenMath(sibling->get_position()));
        if (sibling == m_Selected)
        {
            newJoint->SetIsEndEffector(true);
        }
        m_RotationalJoints.push_back(newJoint);
    }

    if (newJoint->GetChildren().size() > 0)
    {
        int i = 0;
        while (i < newJoint->GetChildren().size())
        {
            effectorBranch(newJoint->GetChildren()[i]);
            i++;
        }
    }
}

void A2Solution::constructTreeFromRoot(Joint2D* joint)
{
    Joint2D* hierarchy_root = joint;
    while (!hierarchy_root->get_parents().empty())
    {
        hierarchy_root = hierarchy_root->get_parents()[0];

        if (hierarchy_root->is_locked())
            break;
    }

    effectorBranch(hierarchy_root);
}

MatrixXd A2Solution::jacobian(std::vector<Joint*> end_effectors)
{
    Vector3d axis_of_rotation = Vector3d::UnitZ();

    int num_columns = m_RotationalJoints.size();
    int num_rows = 2 * end_effectors.size();
    MatrixXd jac(num_rows, num_columns);

    // Go through the columns of J
    for (int col = 0; col < num_columns; col++)
    {
        Vector3d joint_vectorForm(3);
        joint_vectorForm(0) =  m_RotationalJoints[col]->GetPosition().x();
        joint_vectorForm(1) =  m_RotationalJoints[col]->GetPosition().y();
        joint_vectorForm(2) = 0;

        std::vector<Vector3d> end_effectors_vector;
        for (int i = 0; i < end_effectors.size(); i++)
        {
            Vector3d end_effector_vectorForm(3);
            end_effector_vectorForm(0) = end_effectors[i]->GetPosition().x();
            end_effector_vectorForm(1) = end_effectors[i]->GetPosition().y();
            end_effector_vectorForm(2) = 0;

            if (!is_descendant_of(end_effectors[i]->GetJoint(), m_RotationalJoints[col]->GetJoint()))
            {
                end_effectors_vector.push_back(Vector3d(0, 0, 0));
            }
            else
            {
                end_effectors_vector.push_back((-axis_of_rotation.cross(end_effector_vectorForm - joint_vectorForm)).normalized());
            }
        }

        for (int row = 0; row < num_rows; row += 2)
        {
            jac(row, col) = end_effectors_vector[row].x();
            jac(row + 1, col) = end_effectors_vector[row].y();
        }
    }

    return jac;
}

bool A2Solution::is_descendant_of(Joint2D* child, Joint2D* ancestor)
{
    while (!child->get_parents().empty())
    {
        if (child->get_parents()[0] == ancestor)
            return true;

        child = child->get_parents()[0];
    }

    return false;
}

VectorXd A2Solution::dampedLeastSquares(MatrixXd jac, float damping_factor, VectorXd error_vector)
{
    MatrixXd transposeJac = jac.transpose();

    MatrixXd lambda_I(jac.rows(), jac.rows());
    lambda_I = lambda_I.setIdentity() * (damping_factor * damping_factor);

    MatrixXd changeInAngles = transposeJac * ((jac * transposeJac) + lambda_I).inverse();

    return changeInAngles * error_vector;
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

void A2Solution::clearData()
{
    for (auto p : m_Joints)
    {
      delete p;
    }
    m_Joints.clear();
    for (auto p : m_RotationalJoints)
    {
      delete p;
    }
    m_RotationalJoints.clear();
    for (auto p : m_EffectorJoints)
    {
      delete p;
    }
    m_EffectorJoints.clear();
}

