# Inverse-Kinematics
simple inverse kinematics simulator for Computer Animations

###What is Inverse Kinematics (IK)?
IK, much like Forward Kinematics, is a behaviour that is used in animation softwares, such as Blender, MAYA, Flash 8 (for 2D), and more. It is also used in game engines to predict player movements, in robotics and even in biology (protein folding)! Inverse Kinematics is used for many of the same reasons as FK, but adds much functionality to a body's rig. This is because, instead of projecting the new positions of child joints from changes done to their parent, IK does the inverse! It will in fact use the user's modifications from a child joint (called end-effector) and adjust all of its parent's joints to realistically fit the new position.

###How does Forward Kinematics (FK) work?
IK has more steps than FK.

###In Practice
1. Find the jacobian matrix of the partial derives of the end-effector's position with respect to its change in angle:
This can be done using this equation, taken from this [paper](http://math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf)

'![equation]()'
**v** = axis of rotation (in 2D this will be the z axis, or the vector <0, 0, 1>)
**s** = end-effector's position
**p** = end-effector ancestor joint positions
'''
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
'''
2. Find the changes in angles (delta theta) that will eventually be applied to each joint in the tree using the jacobian.
This can be done in multiple ways, but I used the Damping Least Squares method:
'![damping]()'

**J** = jacobian matrix
**lambda** = damping factor
**I** = identity matrix
**e** = error vector, or normalized vector from end-effector to goal

'''
VectorXd A2Solution::dampedLeastSquares(MatrixXd jac, float damping_factor, VectorXd error_vector)
{
    MatrixXd transposeJac = jac.transpose();

    MatrixXd lambda_I(jac.rows(), jac.rows());
    lambda_I = lambda_I.setIdentity() * (damping_factor * damping_factor);

    MatrixXd changeInAngles = transposeJac * ((jac * transposeJac) + lambda_I).inverse();

    return changeInAngles * error_vector;
}
'''
3. Go through each joint using a loop
'''
for (int i = 1; i < m_RotationalJoints.size(); i++)
'''
-(a) Update the rotational matrix using the appropriate delta theta angle coresponding to a joint
'''
double angle = 0;

if (is_descendant_of(m_EffectorJoints[0]->GetJoint(), m_RotationalJoints[i]->GetJoint()) || m_RotationalJoints[i]->GetIsEndEffector())
    angle = angle_changes_vector[i-1];

m_RotationMatrix = Matrix3d::Identity();
m_RotationMatrix(0, 0) = cos(angle);
m_RotationMatrix(0, 1) = sin(angle);
m_RotationMatrix(1, 0) = -sin(angle);
m_RotationMatrix(1, 1) = cos(angle);
'''
-(b) Do a FK pass from this joint to all of its children to find their new positions that will be "stored".
'''
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
'''
-(c) Repeat process for next joint
4. Update the new positions of all joints using the "stored" new positions mentionned in step 3.b.
'''
 for (int i = 0; i < m_RotationalJoints.size(); i++)
        {
            m_RotationalJoints[i]->GetJoint()->set_position(convertToQtMath(m_RotationalJoints[i]->GetPosition()));
        }
'''
