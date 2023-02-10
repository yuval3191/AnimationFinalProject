#include <GL.h>
#include "AnimationVisitor.h"
#include "Visitor.h"
#include "Scene.h"
#include "Movable.h"
#include "DebugHacks.h"

void cg3d::AnimationVisitor::Run(Scene* _scene, Camera* camera)
{
	Visitor::Run(scene = _scene, camera);
}

void cg3d::AnimationVisitor::Visit(Model* model)
{
	Eigen::MatrixX3f system = model->GetRotation();
	if (scene->isAnimate()) {
		if (model->name == std::string("first")) {
			model->TranslateInSystem(system, Eigen::Vector3f(0, 0, 0.01f));
		}
	}
}
