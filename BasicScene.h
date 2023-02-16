#pragma once
#include "SceneWithImGui.h"
#include "Movement.cpp"
#include <utility>
#include "CollisionDetection.cpp"
#include "AABB.h"


class BasicScene : public cg3d::SceneWithImGui
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : SceneWithImGui(std::move(name), display) {};
    void BuildImGui();
    void Init(float fov, int width, int height, float near1, float far1);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void MouseCallback(cg3d::Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[]) override;
    void ScrollCallback(cg3d::Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[]) override;
    void CursorPosCallback(cg3d::Viewport* viewport, int x, int y, bool dragging, int* buttonState)  override;
    void KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void AddViewportCallback(cg3d::Viewport* _viewport) override;
    void ResetToLevel(int level, int lives);
    void reArangeObjects(int level);
    cg3d::Viewport* viewport = nullptr;
    std::vector<std::shared_ptr<cg3d::Model>> cyls;
    std::vector<std::shared_ptr<cg3d::Model>> axis;
    std::vector<std::shared_ptr<cg3d::Model>> objs;
    igl::AABB<MatrixXd, 3> cylTree;
    int score = 0;
    int currLevel = 1;
    float speed = 0.01;
    int lives = 3;
    int numOfCyl = 1;
    int cylToMove;
    float scaleFactor = 1;
    void initAll();
    Eigen::Vector3f getSpherePosition();
    bool isReachable();
    Eigen::Vector3f GenerateRandomPoint(std::shared_ptr<cg3d::Camera> camera, float near1, float far1);
    std::vector<igl::AABB<Eigen::MatrixXd, 3>> collisionTrees;
    std::shared_ptr<cg3d::Model> cube1, cube2, cylinder, sphere, sphere2, cube, sp2;
    std::shared_ptr<Movable> root;
    int pickedIndex = 0;
    int tipIndex = 0;
    bool nextLevelFlag = false;
    Eigen::VectorXi EMAP;
    Eigen::MatrixXi F, E, EF, EI;
    Eigen::VectorXi EQ;
    Eigen::MatrixXd V, C, N, T, points, edges, colors;
    std::shared_ptr<cg3d::Material> material;
    std::shared_ptr<cg3d::Material> material1;
    std::shared_ptr<cg3d::Mesh> sphereMesh;
    std::shared_ptr<cg3d::Mesh> cubeMesh;
    std::shared_ptr<cg3d::Mesh> cylMesh;
    std::shared_ptr<cg3d::Mesh> coordsys;
    int lastx = -1, lasty = -1;
    void initObjs(int level);
    int DISPLAY_WIDTH = 0;
    int DISPLAY_HEIGHT = 0;
    Eigen::Affine3f otherPickedToutAtPress;
    //std::vector<std::shared_ptr<cg3d::Model>> objs;
    enum gameMode
    {
        StartMenu,
        Start,
        Easy,
        Medium,
        Hard,
        Pause,
        Choose
    };
    gameMode gameMode = gameMode::StartMenu;



private:
    
};
