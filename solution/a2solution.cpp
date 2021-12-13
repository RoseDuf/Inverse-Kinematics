
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

        m_Selected = selected;
        m_MousePos = mouse_pos;

        //QVector2D newMousePosition = FindNewPosition(mouse_pos);

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
         construct_joint_hierarchy_tree(m_Selected);
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
            apply_rotation(m_RotationalJoints[i]->GetJoint(), parent_to_joint, pJoint);

            //5. Update new position from FK transformations
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
     }
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

void A2Solution::construct_joint_hierarchy_tree(Joint2D* joint)
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

void A2Solution::apply_rotation(Joint2D* joint, Vector2d parent_to_joint, Vector2d parent_pos)
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
                apply_rotation(newJoint->GetChildren()[i], pJoint, newJoint->GetPosition());
            }

            i++;
        }
    }
}

Matrix3d A2Solution::translationTransformation(Joint2D* joint, QVector2D mouse_pos)
{
    Vector2d translVector = convertToEigenMath(joint->get_position()) - convertToEigenMath(mouse_pos);
    Matrix3d translationMatrix = Matrix3d::Identity();
    translationMatrix.col(2) = Vector3d(translVector.x(), translVector.y(), 1);

    return translationMatrix;
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

QVector2D A2Solution::FindNewPosition(QVector2D mouse_pos)
{
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

    return newMousePosition;
}

/*
///
/// Author: Daniel Rinaldi
/// SID: 40010464
///

#include "a2solution.h"

#include <QDebug>
#include <QApplication>
#include <QWidget>

#include <queue>
#include <algorithm>

A2Solution::A2Solution(std::vector<Joint2D*>& joints, std::vector<Link2D*>& links, std::vector<Obstacle2D*>& obstacles):
    m_joints(joints), m_links(links), m_obstacles(obstacles), num_joints(0), initialized(false), parent_joint_theta_angles(), hierarchy_joint_indexes()
{
    //
}

A2Solution::~A2Solution()
{
}

void A2Solution::update(Joint2D* selected, QVector2D mouse_pos)
{
    if (!initialized)
        init();

    construct_joint_hierarchy_tree(selected);

    // if we are just dragging the root, then just translate using FK
    if (selected->get_parents().empty())
        apply_translation(selected, QtToEigen(mouse_pos - selected->get_position()));

    apply_inverse_kinematics(selected, mouse_pos);

    update_scene();
}

void A2Solution::init()
{
    QWidget* window = QApplication::activeWindow();
    window->setWindowTitle("COMP 477 A2");

    initialized = true;
}

void A2Solution::apply_inverse_kinematics(Joint2D* joint, QVector2D destination)
{
    for (int i = 0; i < MAX_ITERATIONS; ++i)
    {
        MatrixXf J = get_jacobian(joint);
        // e (error) is a vector from the end_effector to the mouse_pos
        // if the error vector is not getting smaller than there is a problem somewhere
        VectorXf e = QtToEigen(destination) - hierarchy_joint_positions[hierarchy_joint_indexes[joint]];
        // clamp/downscale error vector
        if (e.norm() != 0)
            e *= (std::min(beta, e.norm()) / e.norm());

        // Damped Least Squares (DLS)
        // delta_theta = J.transpose() * (J * J.transpose() + lambda^2 * MatrixXf::Identity())^(-1) * e;
        MatrixXf dlsJ = dls(J, lambda);
        VectorXf delta_theta = dlsJ * e;

        // map delta_theta to joints
        parent_joint_theta_angles.clear();
        for (int i = 0; i < joints_in_hierarchy.size(); ++i)
            parent_joint_theta_angles[joints_in_hierarchy[i]] = delta_theta[i];

        fk_pass();
    }
}

void A2Solution::construct_joint_hierarchy_tree(Joint2D* end_effector)
{
    // find hierarchy root
    hierarchy_root = end_effector;
    while (!hierarchy_root->get_parents().empty())
    {
        hierarchy_root = hierarchy_root->get_parents()[0];

        if (hierarchy_root->is_locked())
            break;
    }

    joints_in_hierarchy.clear();
    hierarchy_joint_positions.clear();

    // construct joint hierarchy tree (start from root)
    joints_in_hierarchy.push_back(hierarchy_root);
    hierarchy_joint_positions.push_back(QtToEigen(hierarchy_root->get_position()));
    hierarchy_joint_indexes[hierarchy_root] = joints_in_hierarchy.size() - 1;
    if (hierarchy_root == end_effector) end_effector_index = 0;

    std::deque<Joint2D*> children;
    for (Joint2D* child : hierarchy_root->get_children())
        children.push_back(child);

    Joint2D* current;
    while (children.size() > 0)
    {
        current = children[0];

        for (Joint2D* child : current->get_children())
            children.push_back(child);

        joints_in_hierarchy.push_back(current);
        hierarchy_joint_positions.push_back(QtToEigen(current->get_position()));
        hierarchy_joint_indexes[current] = joints_in_hierarchy.size() - 1;
        if (current == end_effector) end_effector_index = joints_in_hierarchy.size()-1;

        children.pop_front();
    }
}

MatrixXf A2Solution::get_jacobian(Joint2D* end_effector)
{
    int num_cols = joints_in_hierarchy.size();
    // rows = 2 * num_end_effectors; we assume there is only 1 end_effector, so 2*1=2
    int num_rows = 2;
    MatrixXf J(num_rows, num_cols);

    for (int c = 0; c < num_cols; ++c)
    {
        // get the joint that this column represents
        Joint2D* joint = joints_in_hierarchy[c];
        // Does rotating this joint move our end_effector?
        // if end_effector is not a child of joint, then it will not directly move the end_effector.
        if (!is_descendant_of(end_effector, joint))
        {
            J(0,c) = 0;
            J(1,c) = 0;
        }
        else
        {
            // delta = vj.cross(ee - joint)
            Vector2f joint_to_ee = hierarchy_joint_positions[end_effector_index] - hierarchy_joint_positions[c];
            Vector3f entry = -(Vector3f::UnitZ()).cross(Vector3f(joint_to_ee.x(), joint_to_ee.y(), 0));
            entry.normalize();
            J(0,c) = entry[0];
            J(1,c) = entry[1];
        }
    }

    return J;
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

void A2Solution::fk_pass()
{
    for (int i = 0; i < joints_in_hierarchy.size(); ++i)
    {
        if (joints_in_hierarchy[i] == hierarchy_root) continue;

        float angle = 0;
        if (is_descendant_of(joints_in_hierarchy[end_effector_index], joints_in_hierarchy[i]) || joints_in_hierarchy[end_effector_index] == joints_in_hierarchy[i])
            angle = parent_joint_theta_angles[joints_in_hierarchy[i]->get_parents()[0]];

        int parent_index = hierarchy_joint_indexes[joints_in_hierarchy[i]->get_parents()[0]];
        Vector2f parent_to_joint = hierarchy_joint_positions[i] - hierarchy_joint_positions[parent_index];
        apply_rotation(joints_in_hierarchy[i], parent_to_joint, angle, false);
    }
}

void A2Solution::apply_translation(Joint2D* joint, Vector2f translation_vec)
{
    Matrix3f T;
    T << 1,0,translation_vec.x(), 0,1,translation_vec.y(), 0,0,1;

    Vector2f pos2D = QtToEigen(joint->get_position());
    Vector3f new_pos = T * Vector3f(pos2D.x(), pos2D.y(), 1);
    Vector2f new_pos_2d = Vector2f(new_pos.x(), new_pos.y());

    Vector2f parent_pos(0, 0);
    if (!joint->get_parents().empty())
        parent_pos = QtToEigen(joint->get_parents()[0]->get_position());

    hierarchy_joint_positions[hierarchy_joint_indexes[joint]] = new_pos_2d;

    for (Joint2D* child : joint->get_children())
        apply_translation(child, translation_vec);
}

void A2Solution::apply_rotation(Joint2D* joint, Vector2f parent_to_joint, float angle, bool propagate)
{
    Matrix3f R;
    R << std::cos(angle), std::sin(angle), 0,
         -std::sin(angle), std::cos(angle), 0,
         0, 0, 1;

    int parent_index = hierarchy_joint_indexes[joint->get_parents()[0]];
    Vector2f parent_pos = hierarchy_joint_positions[parent_index];
    Vector3f new_pos = (R * Vector3f(parent_to_joint.x(), parent_to_joint.y(), 1)) + Vector3f(parent_pos.x(), parent_pos.y(), 1);
    Vector2f new_pos_2d = Vector2f(new_pos.x(), new_pos.y());
    Vector2f old_pos = hierarchy_joint_positions[hierarchy_joint_indexes[joint]];

    hierarchy_joint_positions[hierarchy_joint_indexes[joint]] = new_pos_2d;

    for (Joint2D* child : joint->get_children())
    {
        apply_rotation(child, hierarchy_joint_positions[hierarchy_joint_indexes[child]] - old_pos, angle, propagate);
    }
}

float A2Solution::compute_theta(Vector2f vec1, Vector2f vec2)
{
    float v1_angle = std::atan2(-vec1.y(), vec1.x());
    float v2_angle = std::atan2(-vec2.y(), vec2.x());
    return v2_angle - v1_angle;
}

void A2Solution::update_scene()
{
    for (int i = 0; i < joints_in_hierarchy.size(); ++i)
    {
        bool bone_is_colliding = false;

        Vector2f parent_new_pos = joints_in_hierarchy[i] == hierarchy_root ?
            hierarchy_joint_positions[i] :
            hierarchy_joint_positions[hierarchy_joint_indexes[joints_in_hierarchy[i]->get_parents()[0]]];

        bone_is_colliding = is_bone_colliding(parent_new_pos, hierarchy_joint_positions[i], joints_in_hierarchy[i]->get_radius());
        if (bone_is_colliding)
            return; // abort
    }

    for (int i = 0; i < joints_in_hierarchy.size(); ++i)
    {
        joints_in_hierarchy[i]->set_position(EigenToQt(hierarchy_joint_positions[i]));
    }
}

bool A2Solution::is_bone_colliding(Vector2f joint_pos_a, Vector2f joint_pos_b, float joint_radius)
{
    for (Obstacle2D* obstacle : m_obstacles)
    {
        if (joint_pos_a != joint_pos_b)
        {
            Vector2f circle_pos = QtToEigen(obstacle->m_center);
            Vector2f closest_point_on_bone = compute_closest_point_on_bone(joint_pos_a, joint_pos_b, circle_pos);
            if ((closest_point_on_bone - circle_pos).norm() - obstacle->m_radius - bone_thickness < 0)
                return true;
        }

        Vector2f joint_to_obstacle = QtToEigen(obstacle->m_center) - joint_pos_a;
        if (joint_to_obstacle.norm() - joint_radius - obstacle->m_radius < 0)
            return true;

        joint_to_obstacle = QtToEigen(obstacle->m_center) - joint_pos_b;
        if (joint_to_obstacle.norm() - joint_radius - obstacle->m_radius < 0)
            return true;
    }

    return false;
}

Vector2f A2Solution::compute_closest_point_on_bone(Vector2f joint_pos_a, Vector2f joint_pos_b, Vector2f obstacle_pos)
{
    Vector2f ab = joint_pos_b - joint_pos_a;
    Vector2f ao = obstacle_pos - joint_pos_a;
    Vector2f p = joint_pos_a + (ao.dot(ab)/ab.dot(ab)) * ab;

    float ab_norm = (joint_pos_a - joint_pos_b).norm();
    float ap_norm = (joint_pos_a - p).norm();
    float pb_norm = (p - joint_pos_b).norm();
    if (ap_norm > ab_norm || pb_norm > ab_norm)
    {
        if ((joint_pos_a - obstacle_pos).norm() < (joint_pos_b - obstacle_pos).norm())
            return joint_pos_a;
        else
            return joint_pos_b;
    }

    return p;
}

MatrixXf A2Solution::dls(MatrixXf J, float lam)
{
    MatrixXf JJt = J * J.transpose();
    MatrixXf lambda_I(J.rows(), J.rows());
    lambda_I = lambda_I.setIdentity() * (lam*lam);
    return J.transpose() * (JJt + lambda_I).inverse();
}

namespace
{
    Vector2f QtToEigen(QVector2D q_vec)
    {
        return Vector2f(q_vec.x(), -q_vec.y());
    }

    QVector2D EigenToQt(Vector2f vec2f)
    {
        return QVector2D(vec2f.x(), -vec2f.y());
    }
}

void A2Solution::test_eigen_library(){

    // create a simple matrix 5 by 6
    MatrixXd mat(5,6);

    // Fills in matrix
    for(int row=0;row<mat.rows();row++){
        for(int col=0;col<mat.cols();col++){
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
*/

