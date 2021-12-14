
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
    void applyTransformationsIK(QVector2D mouse_pos);
    Vector2d distanceFromGoal(QVector2D goal, std::vector<Joint*> e_joints);
    void effectorBranch(Joint2D* end_effector);
    void constructTreeFromRoot(Joint2D* sibling);
    void RotationTransformations(Joint2D* joint, Vector2d parent_to_joint, Vector2d parent_pos);
    void TranslationTransformations(Joint2D* joint, Vector2d translation_vec);
    void updatePositions();
    void Traverse(Joint2D* sibling, Matrix3d worldTransf);
    Vector2d convertToEigenMath(QVector2D vector);
    QVector2D convertToQtMath(Vector2d vector);
    MatrixXd jacobian(std::vector<Joint*> end_effectors);
    bool is_descendant_of(Joint2D* child, Joint2D* ancestor);
    VectorXd dampedLeastSquares(MatrixXd jac, float damping_factor, VectorXd error_vector);
    void clearData();

    bool m_IsInitialized;
    std::vector<Joint2D*> m_CurrentJoints;
    std::vector<Joint*> m_RotationalJoints;
    std::vector<Joint*> m_EffectorJoints;
    std::vector<Joint*> m_Joints;

    Joint2D* m_Selected;
    Matrix3d m_RotationMatrix = Matrix3d::Identity();
    Matrix3d m_TranslationMatrix = Matrix3d::Identity();
    Matrix3d m_RotationTransf = Matrix3d::Identity();
};

#endif
