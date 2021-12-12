
#ifndef A2SOLUTION_H
#define A2SOLUTION_H

#include <vector>
#include <queue>
#include <QDebug>
#include <math.h>

#include "OpenGL/elements/joint2d.h"
#include "OpenGL/elements/obstacle2d.h"
#include "OpenGL/elements/link2d.h"
#include "Joint.h"

using Eigen::Vector3f;
using Eigen::Vector3d;
using Eigen::VectorXf;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Rotation2D;
using Eigen::atan;
using Eigen::cos;
using Eigen::sin;

class A2Solution
{
public:
    A2Solution(std::vector<Joint2D*>& joints, std::vector<Link2D*>& links, std::vector<Obstacle2D*>& obstacles);
    ~A2Solution();

    std::vector<Joint2D*>& m_joints;
    std::vector<Link2D*>& m_links;
    std::vector<Obstacle2D*>& m_obstacles;

    void update(Joint2D* selected, QVector2D mouse_pos);
    static void test_eigen_library();

private:
    void initialize();
    void updatePositionsInUI();
    bool changesWereMade();
    void createTree(std::vector<Joint2D*> joints);
    void applyTransformationsIK(QVector2D mouse_pos);
    void applyTransformations(QVector2D mouse_pos);
    Vector2d distanceFromGoal(QVector2D goal, std::vector<Joint*> e_joints);
    void effectorBranch(Joint2D* end_effector);
    void construct_joint_hierarchy_tree(Joint2D* sibling);
    Matrix3d rotationTransformation(Matrix3d rotationMatrix, Joint* joint);
    Matrix3d translationTransformation(Joint2D* joint, QVector2D mouse_pos);
    void apply_rotation(Joint2D* joint, Vector2d parent_to_joint, Vector2d parent_pos);
    void Traverse(Joint2D* sibling, Matrix3d worldTransf);
    Vector2d convertToEigenMath(QVector2D vector);
    QVector2D convertToQtMath(Vector2d vector);
    float getRotationAngle(Joint2D* joint, QVector2D new_pos);
    MatrixXd jacobian(std::vector<Joint*> end_effectors);
    bool is_descendant_of(Joint2D* child, Joint2D* ancestor);
    VectorXd dampedLeastSquares(MatrixXd jac, float damping_factor, VectorXd error_vector);

    QVector2D FindNewPosition(QVector2D mouse_pos);

    bool m_IsInitialized;
    std::vector<Joint2D*> m_CurrentJoints;
    std::vector<Joint*> m_RotationalJoints;
    std::vector<Joint*> m_EffectorJoints;
    std::vector<Joint*> m_Joints;
    Vector2d m_OriginalPosition;

    Joint2D* m_Selected;
    int m_selectedIndex;
    QVector2D m_MousePos;

    Matrix3d m_WorldTransf = Matrix3d::Identity();
    Joint2D* m_ParentJoint = nullptr;
    Matrix3d m_RotationMatrix = Matrix3d::Identity();
    Matrix3d m_TranslationTransf = Matrix3d::Identity();
    Matrix3d m_RotationTransf = Matrix3d::Identity();

    float m_Magnitude;
};

#endif // A2SOLUTION_H
/*
///
/// Author: Daniel Rinaldi
/// SID: 40010464
///

#ifndef A2SOLUTION_H
#define A2SOLUTION_H

#include <vector>
#include <map>

#include "OpenGL/elements/joint2d.h"
#include "OpenGL/elements/obstacle2d.h"
#include "OpenGL/elements/link2d.h"

#include "dependencies/Eigen/Dense"

using std::vector;
using Eigen::VectorXf;
using Eigen::Vector3f;
using Eigen::Vector2f;
using Eigen::Rotation2D;
using Eigen::MatrixXf;
using Eigen::Matrix3f;
using Eigen::MatrixXd;

class A2Solution
{
public:
    A2Solution(std::vector<Joint2D*>& joints, std::vector<Link2D*>& links, std::vector<Obstacle2D*>& obstacles);
    ~A2Solution();

    std::vector<Joint2D*>& m_joints;
    std::vector<Link2D*>& m_links;
    std::vector<Obstacle2D*>& m_obstacles;

    void update(Joint2D* selected, QVector2D mouse_pos);

    static void test_eigen_library();

private:
    const int MAX_ITERATIONS = 50;
    float beta = 1.0f;
    float lambda = 12;
    float bone_thickness = 2;
    float d_ta = 30;
    float d_ug = 60;
    float d_soi = 90;
    float a_n = 0.01f;
    float a_o = 0.01f;
    float obs_lambda = 40;

    int num_joints;
    bool initialized;
    Joint2D* hierarchy_root;
    vector<Joint2D*> joints_in_hierarchy;
    vector<Vector2f> hierarchy_joint_positions;
    int end_effector_index;
    std::map<Joint2D*, float> parent_joint_theta_angles;
    std::map<Joint2D*, int> hierarchy_joint_indexes;

    void init();
    void apply_inverse_kinematics(Joint2D* joint, QVector2D destination);
    void construct_joint_hierarchy_tree(Joint2D* end_effector);
    MatrixXf get_jacobian(Joint2D* end_effector);
    bool is_descendant_of(Joint2D* child, Joint2D* ancestor);
    void fk_pass();
    void apply_translation(Joint2D* joint, Vector2f translation_vec);
    void apply_rotation(Joint2D* joint, Vector2f parent_to_joint, float theta, bool propagate);
    float compute_theta(Vector2f vec1, Vector2f vec2);
    void update_scene();
    bool is_bone_colliding(Vector2f joint_pos_a, Vector2f joint_pos_b, float joint_radius);
    Vector2f compute_closest_point_on_bone(Vector2f joint_pos_a, Vector2f joint_pos_b, Vector2f obsacle_pos);
    MatrixXf dls(MatrixXf J, float lam);
};

namespace
{
    Vector2f QtToEigen(QVector2D qvec);
    QVector2D EigenToQt(Vector2f vec2f);
}

#endif // A2SOLUTION_H

*/
