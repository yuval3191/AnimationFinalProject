#include "windows.h"
#undef CreateWindow
#pragma comment(lib,"winmm.lib")
#include <GL.h>
#include "AnimationVisitor.h"
#include "Visitor.h"
#include "Scene.h"
#include "Movable.h"
#include "DebugHacks.h"
#include "../../tutorial/YuvalNoamOmri/BasicScene.h"

void cg3d::AnimationVisitor::Run(Scene* _scene, Camera* camera)
{
	Visitor::Run(scene = _scene, camera);
}

void cg3d::AnimationVisitor::Visit(Model* model)
{
	Eigen::MatrixX3f system = Eigen::Affine3f(model->GetAggregatedTransform()).rotation().transpose();
	//Eigen::MatrixX3f system = model->GetRotation().transpose();
	if (scene->isAnimate()) {
		BasicScene* ourScene = (BasicScene*)scene;
		if (model->name == std::string("first")) {
			//Eigen::MatrixX3f system = ourScene->firstPersonCamera->GetRotation().transpose();
			for (int i = 0; i < ourScene->objs.size(); i++)
			{
				if (CollisionDetection::CheckCollisionDetection(
					ourScene->cylTree,
					ourScene->collisionTrees[i],
					ourScene->objs[i],
					ourScene->cyls[0]))
				{
					if (ourScene->objs[i]->name == "Sphere") {
						ourScene->score = ourScene->score + 10;
						auto newPoint = ourScene->GenerateRandomPoint(ourScene->camera, 5, 35);
						ourScene->objs[i]->SetTransform(Eigen::Matrix4f::Identity());
						ourScene->objs[i]->Translate(newPoint);
						ourScene->animate = true;
						PlaySound("C:\\Users\\User\\Desktop\\yay.wav", NULL, SND_ASYNC);
						if (ourScene->score % 20 == 0) {
							ourScene->nextLevelFlag = true;
							PlaySound("C:\\Users\\User\\Desktop\\finishLevel.wav", NULL, SND_ASYNC);
						}
					}
					else {
						PlaySound("C:\\Users\\User\\Desktop\\music2.wav", NULL, SND_ASYNC);
						ourScene->animate = false;
						ourScene->lives = ourScene->lives - 1;
						ourScene->ResetToLevel(ourScene->currLevel, ourScene->lives);
					}
				}
				else {
					Vector3f end = getEndpoint(0, { 0,0,0.8f }, ourScene->cyls);
					Vector3f start = getEndpoint(0, { 0,0,-0.8f }, ourScene->cyls);
					Vector3f direction = (end - start).normalized();
					//model->TranslateInSystem(system, { 0,0,ourScene->speed });
					model->Translate(direction * ourScene->speed);
				}
			}
		}
	}
}
