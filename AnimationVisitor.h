#pragma once

#include "Visitor.h"
#include "Camera.h"

#include <utility>
#include <memory>
#include "../../tutorial/YuvalNoamOmri/Movement.cpp"
#include "../../tutorial/YuvalNoamOmri/CollisionDetection.cpp"

namespace cg3d
{

    class AnimationVisitor : public Visitor
    {
    public:
        void Run(Scene* scene, Camera* camera) override;
        void Visit(Model* model) override;
        // void Init() override;
        bool drawOutline = true;
        float outlineLineWidth = 5;
        Eigen::Vector3f oldTailPos;
        Eigen::Vector3f basePos;

        bool firstTime = true;
        Eigen::Vector4f outlineLineColor{ 1, 1, 1, 1 };

    private:
        void DrawOutline();

        Scene* scene;
    };

} // namespace cg3d

