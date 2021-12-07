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

using Eigen::Vector2d;
using Eigen::Vector3f;
using Eigen::Vector3d;
using Eigen::VectorXf;
using Eigen::VectorXd;
using Eigen::MatrixXd;
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
    Vector2d distanceFromGoal(QVector2D goal);
    void effectorBranch(Joint2D* end_effector);
    Matrix3d rotationTransformation(Matrix3d rotationMatrix, Joint* joint);
    Matrix3d translationTransformation(Joint2D* joint, QVector2D mouse_pos);
    void Traverse(Joint2D* sibling, Matrix3d worldTransf);
    Vector2d convertToEigenMath(QVector2D vector);
    QVector2D convertToQtMath(Vector2d vector);
    float getRotationAngle(Joint2D* joint, QVector2D new_pos);
    MatrixXd jacobian(std::vector<Joint*> joints, std::vector<Joint*> end_effectors);
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
