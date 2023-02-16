#pragma once

#include <utility>
#include <Eigen/Core>
#include <Eigen/LU>
#include <igl/opengl/glfw/Viewer.h>
#include <iostream>
#include "../engine/Mesh.h"
#include <igl/AABB.h>
#include <GLFW/glfw3.h>
#include <igl/per_vertex_normals.h>
#include <Movable.h>
#include <Model.h>
#include <queue>

#define Pair std::pair<igl::AABB< Eigen::MatrixXd, 3>, igl::AABB< Eigen::MatrixXd, 3>>

/*

	Helper functions for collision detection

*/
using namespace Eigen;;

namespace CollisionDetection {

    static std::shared_ptr<cg3d::Mesh> meshifyBoundingBox(Eigen::AlignedBox3d &box) {

        Eigen::Vector3d
                blf = box.corner(Eigen::AlignedBox3d::BottomLeftFloor),
                brf = box.corner(Eigen::AlignedBox3d::BottomRightFloor),
                blc = box.corner(Eigen::AlignedBox3d::BottomLeftCeil),
                brc = box.corner(Eigen::AlignedBox3d::BottomRightCeil),
                tlf = box.corner(Eigen::AlignedBox3d::TopLeftFloor),
                trf = box.corner(Eigen::AlignedBox3d::TopRightFloor),
                tlc = box.corner(Eigen::AlignedBox3d::TopLeftCeil),
                trc = box.corner(Eigen::AlignedBox3d::TopRightCeil);
        Eigen::MatrixXd V(8, 3);
        V.row(0) = trc;
        V.row(1) = tlc;
        V.row(2) = blc;
        V.row(3) = brc;
        V.row(4) = trf;
        V.row(5) = tlf;
        V.row(6) = blf;
        V.row(7) = brf;


        Eigen::MatrixXi F(12, 3);

        F.row(0) = Eigen::Vector3i(0, 1, 2);
        F.row(1) = Eigen::Vector3i(0, 2, 3);
        F.row(2) = Eigen::Vector3i(7, 4, 0);
        F.row(3) = Eigen::Vector3i(7, 0, 3);
        F.row(4) = Eigen::Vector3i(4, 5, 1);
        F.row(5) = Eigen::Vector3i(4, 1, 0);
        F.row(6) = Eigen::Vector3i(5, 6, 2);
        F.row(7) = Eigen::Vector3i(5, 2, 1);
        F.row(8) = Eigen::Vector3i(3, 2, 6);
        F.row(9) = Eigen::Vector3i(3, 6, 7);
        F.row(10) = Eigen::Vector3i(6, 5, 4);
        F.row(11) = Eigen::Vector3i(6, 4, 7);

        Eigen::MatrixXd N;
        igl::per_vertex_normals(V, F, N);
        return std::make_shared<cg3d::Mesh>(cg3d::Mesh("Bounding box", V, F, N, {}));
    }

    static void calcBoxInSpace(const float &scale,
                               const Eigen::AlignedBox3d &box,
                               const Eigen::Matrix4f &transform,
                               Eigen::Vector3d &C,
                               Eigen::Matrix3d &A,
                               Eigen::Vector3d &a) {
        // calculate C (center) in the global space
        Eigen::Vector4d Cvec4;
        Cvec4 << box.center()[0], box.center()[1], box.center()[2], 1;
        C = (transform.cast<double>() * Cvec4).head(3);
        // calculate A1, A2, A3 axes in the global space
        Eigen::Vector4d A1vec4 = transform.cast<double>() * Eigen::Vector4d(1, 0, 0, 0);
        Eigen::Vector4d A2vec4 = transform.cast<double>() * Eigen::Vector4d(0, 1, 0, 0);
        Eigen::Vector4d A3vec4 = transform.cast<double>() * Eigen::Vector4d(0, 0, 1, 0);
        A.col(0) = A1vec4.head(3).normalized();
        A.col(1) = A2vec4.head(3).normalized();
        A.col(2) = A3vec4.head(3).normalized();
        // calculate a1, a2, a3 extents
        a = (box.sizes() / 2) * scale;
    }

    static bool checkWise(Vector3f t, Vector3f l, double wA, double wB, Vector3f aX, Vector3f aY, Vector3f aZ, Vector3f bX, Vector3f bY, Vector3f bZ, double hA, double hB, double dA, double dB)
    {
        double dotAns = abs(t.dot(l));
        double a1 = abs((wA * aX).dot(l));
        double a2 = abs((hA * aY).dot(l));
        double a3 = abs((dA * aZ).dot(l));
        double b1 = abs((wB * bX).dot(l));
        double b2 = abs((hB * bY).dot(l));
        double b3 = abs((dB * bZ).dot(l));

        return dotAns > a1 + a2 + a3 + b1 + b2 + b3;
    }

    static bool intersects2(
        const igl::AABB<Eigen::MatrixXd, 3>& tree1,
        const igl::AABB<Eigen::MatrixXd, 3>& tree2,
        const std::shared_ptr<cg3d::Model> obj1,
        const std::shared_ptr<cg3d::Model> obj2
    ) {

        Eigen::AlignedBox<double, 3> box1 = tree1.m_box, box2 = tree2.m_box;
        RowVectorXd p1 = box1.center();
        RowVectorXd p2 = box2.center();
        Eigen::Vector4f C0(p1(0), p1(1), p1(2), 1);
        Eigen::Vector4f C1(p2(0), p2(1), p2(2), 1);

        Vector3f ax = obj1->GetRotation().col(0);
        Vector3f ay = obj1->GetRotation().col(1);
        Vector3f az = obj1->GetRotation().col(2);
        Vector3f bx = obj2->GetRotation().col(0);
        Vector3f by = obj2->GetRotation().col(1);
        Vector3f bz = obj2->GetRotation().col(2);
        Matrix4f aTransform = obj1->GetTransform();
        Matrix4f bTransform = obj2->GetTransform();
        Eigen::Vector4f D = (bTransform * C1) - (aTransform * C0);

        double wa = box1.sizes().x();
        double ha = box1.sizes().y();
        double da = box1.sizes().z();
        double wb = box2.sizes().x();
        double hb = box2.sizes().y();
        double db = box2.sizes().z();

        std::vector<Vector3f> cases = { ax, ay, az, bx, by, bz, ax.cross(bx), ax.cross(by), ax.cross(bz), ay.cross(bx), ay.cross(by), ay.cross(bz), az.cross(bx), az.cross(by), az.cross(bz) };
        Vector3f newD = { D.x(), D.y(), D.z() };
        for (Vector3f l : cases)
        {
            if (checkWise(newD, l, wa, wb, ax, ay, az, bx, by, bz, ha, hb, da, db))
            {
                return false;
            }
        }

        if (tree1.is_leaf() || tree2.is_leaf())
        {
            if (tree1.is_leaf() && tree2.is_leaf()) {
               
                return true;
            }
            if (tree1.is_leaf())
            {
                return (intersects2(tree1, *(tree2.m_left), obj1, obj2) || intersects2(tree1, *(tree2.m_right), obj1, obj2));
            }
            else
            {
                return (intersects2(*(tree1.m_left), tree2, obj1, obj2) || intersects2(*(tree1.m_right), tree2, obj1, obj2));
            }

        }
        else
        {
            return (intersects2(*(tree1.m_left), *(tree2.m_left), obj1, obj2) ||
                intersects2(*(tree1.m_left), *(tree2.m_right), obj1, obj2) ||
                intersects2(*(tree1.m_right), *(tree2.m_left), obj1, obj2) ||
                intersects2(*(tree1.m_right), *(tree2.m_right), obj1, obj2));
        }

    }

   

    static bool intersects(
            const float &scale,
            const igl::AABB<Eigen::MatrixXd, 3> &obb0,
            const Eigen::Matrix4f &transform0,
            const igl::AABB<Eigen::MatrixXd, 3> &obb1,
            const Eigen::Matrix4f &transform1
            
    ) {
        Eigen::Vector3d C0, a;
        Eigen::Matrix3d A;
        calcBoxInSpace(scale, obb0.m_box, transform0, C0, A, a);
        Eigen::Vector3d C1, b;
        Eigen::Matrix3d B;
        calcBoxInSpace(scale, obb1.m_box, transform1, C1, B, b);

        Eigen::Matrix3d C = A.transpose() * B;
        Eigen::Vector3d D = C1 - C0;

        double R0;
        double R1;
        double R;

        // L = A0
        R0 = a[0];
        R1 = b[0] * abs(C(0, 0)) + b[1] * abs(C(0, 1)) + b[2] * abs(C(0, 2));
        R = abs(A.col(0).transpose() * D);
        if (R > R0 + R1) return false;

        // L = A1
        R0 = a[1];
        R1 = b[0] * abs(C(1, 0)) + b[1] * abs(C(1, 1)) + b[2] * abs(C(1, 2));
        R = abs(A.col(1).transpose() * D);
        if (R > R0 + R1) return false;

        // L = A2
        R0 = a[2];
        R1 = b[0] * abs(C(2, 0)) + b[1] * abs(C(2, 1)) + b[2] * abs(C(2, 2));
        R = abs(A.col(2).transpose() * D);
        if (R > R0 + R1) return false;

        // L = B0
        R0 = a[0] * abs(C(0, 0)) + a[1] * abs(C(1, 0)) + a[2] * abs(C(2, 0));
        R1 = b[0];
        R = abs(B.col(0).transpose() * D);
        if (R > R0 + R1) return false;

        // L = B1
        R0 = a[0] * abs(C(0, 1)) + a[1] * abs(C(1, 1)) + a[2] * abs(C(2, 1));
        R1 = b[1];
        R = abs(B.col(1).transpose() * D);
        if (R > R0 + R1) return false;

        // L = B2
        R0 = a[0] * abs(C(0, 2)) + a[1] * abs(C(1, 2)) + a[2] * abs(C(2, 2));
        R1 = b[2];
        R = abs(B.col(2).transpose() * D);
        if (R > R0 + R1) return false;

        // L = A0 x B0
        R0 = a[1] * abs(C(2, 0)) + a[2] * abs(C(1, 0));
        R1 = b[1] * abs(C(0, 2)) + b[2] * abs(C(0, 1));
        double arg1 = C(1, 0) * A.col(2).transpose() * D;
        double arg2 = C(2, 0) * A.col(1).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        // L = A0 x B1
        R0 = a[1] * abs(C(2, 1)) + a[2] * abs(C(1, 1));
        R1 = b[1] * abs(C(0, 2)) + b[2] * abs(C(0, 0));
        arg1 = C(1, 1) * A.col(2).transpose() * D;
        arg2 = C(2, 1) * A.col(1).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        // L = A0 x B2
        R0 = a[1] * abs(C(2, 2)) + a[2] * abs(C(1, 2));
        R1 = b[1] * abs(C(0, 1)) + b[2] * abs(C(0, 0));
        arg1 = C(1, 2) * A.col(2).transpose() * D;
        arg2 = C(2, 2) * A.col(1).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        // L = A1 x B0
        R0 = a[0] * abs(C(2, 0)) + a[2] * abs(C(0, 0));
        R1 = b[1] * abs(C(1, 2)) + b[2] * abs(C(1, 1));
        arg1 = C(2, 0) * A.col(0).transpose() * D;
        arg2 = C(0, 0) * A.col(2).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        // L = A1 x B1
        R0 = a[0] * abs(C(2, 1)) + a[2] * abs(C(0, 1));
        R1 = b[0] * abs(C(1, 2)) + b[2] * abs(C(1, 0));
        arg1 = C(2, 1) * A.col(0).transpose() * D;
        arg2 = C(0, 1) * A.col(2).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        // L = A1 x B2
        R0 = a[0] * abs(C(2, 2)) + a[2] * abs(C(0, 2));
        R1 = b[0] * abs(C(1, 1)) + b[1] * abs(C(1, 0));
        arg1 = C(2, 2) * A.col(0).transpose() * D;
        arg2 = C(0, 2) * A.col(2).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        // L = A2 x B0
        R0 = a[0] * abs(C(1, 0)) + a[1] * abs(C(0, 0));
        R1 = b[1] * abs(C(2, 2)) + b[2] * abs(C(2, 1));
        arg1 = C(0, 0) * A.col(1).transpose() * D;
        arg2 = C(1, 0) * A.col(0).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        // L = A2 x B1
        R0 = a[0] * abs(C(1, 1)) + a[1] * abs(C(0, 1));
        R1 = b[0] * abs(C(2, 2)) + b[2] * abs(C(2, 0));
        arg1 = C(0, 1) * A.col(1).transpose() * D;
        arg2 = C(1, 1) * A.col(0).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        // L = A2 x B2
        R0 = a[0] * abs(C(1, 2)) + a[1] * abs(C(0, 2));
        R1 = b[0] * abs(C(2, 1)) + b[1] * abs(C(2, 0));
        arg1 = C(0, 2) * A.col(1).transpose() * D;
        arg2 = C(1, 2) * A.col(0).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        if (obb0.is_leaf() && obb1.is_leaf()) {
            return true;
        }
        if (obb0.m_left == nullptr || obb1.m_left == nullptr) return false;
        return
                intersects(scale, *obb0.m_left, transform0, *obb1.m_left, transform1) ||
                intersects(scale, *obb0.m_left, transform0, *obb1.m_right, transform1) ||
                intersects(scale, *obb0.m_right, transform0, *obb1.m_left, transform1) ||
                intersects(scale, *obb0.m_right, transform0, *obb1.m_right, transform1);
    }

    static Eigen::Vector3d MakeDvector(igl::AABB< Eigen::MatrixXd, 3> t1, igl::AABB< Eigen::MatrixXd, 3> t2,
        const std::shared_ptr<cg3d::Model> obj1,
        const std::shared_ptr<cg3d::Model> obj2)
    {
        Eigen::Vector4d C1, C2;
        auto center1 = t1.m_box.center();
        Eigen::Vector4d cent1 = { center1[0], center1[1], center1[2], 1 };
        C1 = obj1->GetTransform().cast<double>() * cent1;

        auto center2 = t2.m_box.center();
        Eigen::Vector4d cent2 = { center2[0], center2[1], center2[2], 1 };
        C2 = obj2->GetTransform().cast<double>() * cent2;

        return (C2 - C1).head(3);
    }

    static bool intersects3(
        const igl::AABB<Eigen::MatrixXd, 3>& t1,
        const igl::AABB<Eigen::MatrixXd, 3>& t2,
        const std::shared_ptr<cg3d::Model> obj1,
        const std::shared_ptr<cg3d::Model> obj2
    ) {
        Eigen::Vector3d D = MakeDvector(t1, t2, obj1, obj2);

        Eigen::Vector3d A0, A1, A2;
        A0 = obj1->GetRotation().cast<double>().col(0);
        A1 = obj1->GetRotation().cast<double>().col(1);
        A2 = obj1->GetRotation().cast<double>().col(2);

        float a0, a1, a2;
        a0 = t1.m_box.sizes()(0) / 2;
        a1 = t1.m_box.sizes()(1) / 2;
        a2 = t1.m_box.sizes()(2) / 2;

        Eigen::Vector3d B0, B1, B2;
        B0 = obj2->GetRotation().cast<double>().col(0);
        B1 = obj2->GetRotation().cast<double>().col(1);
        B2 = obj2->GetRotation().cast<double>().col(2);

        float b0, b1, b2;
        b0 = t2.m_box.sizes()(0) / 2;
        b1 = t2.m_box.sizes()(1) / 2;
        b2 = t2.m_box.sizes()(2) / 2;

        Eigen::Matrix3d M = obj1->GetRotation().cast<double>().transpose() * obj2->GetRotation().cast<double>();

        float c00, c01, c02,
            c10, c11, c12,
            c20, c21, c22;
        c00 = M.row(0)(0);
        c01 = M.row(0)(1);
        c02 = M.row(0)(2);
        c10 = M.row(1)(0);
        c11 = M.row(1)(1);
        c12 = M.row(1)(2);
        c20 = M.row(2)(0);
        c21 = M.row(2)(1);
        c22 = M.row(2)(2);

        // check all the 15 terms

        // case 1
        auto res1 = abs(A0.transpose() * D);
        if (res1 > a0 + b0 * abs(c00) + b1 * abs(c01) + b2 * abs(c02))
            return false;

        // case 2
        auto res2 = abs(A1.transpose() * D);
        if (res2 > a1 + b0 * abs(c10) + b1 * abs(c11) + b2 * abs(c12))
            return false;

        // case 3
        auto res3 = abs(A2.transpose() * D);
        if (res3 > a2 + b0 * abs(c20) + b1 * abs(c21) + b2 * abs(c22))
            return false;

        // case 4
        auto res4 = abs(B0.transpose() * D);
        if (res4 > b0 + a0 * abs(c00) + a1 * abs(c10) + a2 * abs(c20))
            return false;

        // case 5
        auto res5 = abs(B1.transpose() * D);
        if (res5 > b1 + a0 * abs(c01) + a1 * abs(c11) + a2 * abs(c21))
            return false;

        // case 6
        auto res6 = abs(B2.transpose() * D);
        if (res6 > b2 + a0 * abs(c02) + a1 * abs(c12) + a2 * abs(c22))
            return false;

        // case 7
        float R7 = (c10 * A2.transpose() * D);
        float RR7 = (c20 * A1.transpose() * D);
        float res7 = abs(R7 - RR7);
        if (res7 > a1 * abs(c20) + a2 * abs(c10) + b1 * abs(c02) + b2 * abs(c01))
            return false;

        // case 8
        float R8 = (c11 * A2.transpose() * D);
        float RR8 = (c21 * A1.transpose() * D);
        float res8 = abs(R8 - RR8);
        if (res8 > a1 * abs(c21) + a2 * abs(c11) + b0 * abs(c02) + b2 * abs(c00))
            return false;

        // case 9
        float R9 = (c12 * A2.transpose() * D);
        float RR9 = (c22 * A1.transpose() * D);
        float res9 = abs(R9 - RR9);
        if (res9 > a1 * abs(c22) + a2 * abs(c12) + b0 * abs(c01) + b1 * abs(c00))
            return false;

        // case 10
        float R10 = (c20 * A0.transpose() * D);
        float RR10 = (c00 * A2.transpose() * D);
        float res10 = abs(R10 - RR10);
        if (res10 > a0 * abs(c20) + a2 * abs(c00) + b1 * abs(c12) + b2 * abs(c11))
            return false;

        // case 11
        float R11 = (c21 * A0.transpose() * D);
        float RR11 = (c01 * A2.transpose() * D);
        float res11 = abs(R11 - RR11);
        if (res11 > a0 * abs(c21) + a2 * abs(c01) + b0 * abs(c12) + b2 * abs(c10))
            return false;

        // case 12
        float R12 = (c22 * A0.transpose() * D);
        float RR12 = (c02 * A2.transpose() * D);
        float res12 = abs(R12 - RR12);
        if (res12 > a0 * abs(c22) + a2 * abs(c02) + b0 * abs(c11) + b1 * abs(c10))
            return false;

        // case 13
        float R13 = (c00 * A1.transpose() * D);
        float RR13 = (c10 * A0.transpose() * D);
        float res13 = abs(R13 - RR13);
        if (res13 > a0 * abs(c10) + a1 * abs(c00) + b1 * abs(c22) + b2 * abs(c21))
            return false;

        // case 14
        float R14 = (c01 * A1.transpose() * D);
        float RR14 = (c11 * A0.transpose() * D);
        float res14 = abs(R14 - RR14);
        if (res14 > a0 * abs(c11) + a1 * abs(c01) + b0 * abs(c22) + b2 * abs(c20))
            return false;

        // case 15
        float R15 = (c02 * A1.transpose() * D);
        float RR15 = (c12 * A0.transpose() * D);
        float res15 = abs(R15 - RR15);
        if (res15 > a0 * abs(c12) + a1 * abs(c02) + b0 * abs(c21) + b1 * abs(c20))
            return false;

        return true;
    }
    
    static bool CheckCollisionDetection(igl::AABB< Eigen::MatrixXd, 3> t1, igl::AABB< Eigen::MatrixXd, 3> t2,
        const std::shared_ptr<cg3d::Model> obj1,
        const std::shared_ptr<cg3d::Model> obj2) {
        std::queue<Pair> queue;

        // Initialize with main bounding box for each object
        queue.push(Pair(t1, t2));
        // Now, we need to Repeatedly pull next potential pair off queue and test for intersection
        while (!queue.empty()) {
            // taking the first element from the queue
            Pair firstInQueue = queue.front();
            queue.pop();
            // we need to check if there is an intersection between the boxes
            if (intersects3(firstInQueue.first, firstInQueue.second, obj1, obj2))
            {
                if (firstInQueue.first.is_leaf() && firstInQueue.second.is_leaf())
                {

                    return true;
                }
                else if (firstInQueue.first.is_leaf() && !firstInQueue.second.is_leaf())
                {
                    queue.push(Pair(firstInQueue.first, *firstInQueue.second.m_left));
                    queue.push(Pair(firstInQueue.first, *firstInQueue.second.m_right));
                }
                else if (!firstInQueue.first.is_leaf() && firstInQueue.second.is_leaf())
                {
                    queue.push(Pair(*firstInQueue.first.m_left, firstInQueue.second));
                    queue.push(Pair(*firstInQueue.first.m_right, firstInQueue.second));
                }
                else
                {
                    queue.push(Pair(*firstInQueue.first.m_left, *firstInQueue.second.m_left));
                    queue.push(Pair(*firstInQueue.first.m_left, *firstInQueue.second.m_right));
                    queue.push(Pair(*firstInQueue.first.m_right, *firstInQueue.second.m_left));
                    queue.push(Pair(*firstInQueue.first.m_right, *firstInQueue.second.m_right));
                }
            }
        }
        return false;
    }

}



