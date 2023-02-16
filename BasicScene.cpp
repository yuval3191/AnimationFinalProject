#include "windows.h"
#undef CreateWindow
#pragma comment(lib,"winmm.lib")
#include "BasicScene.h"
#include <Eigen/src/Core/Matrix.h>
#include <edges.h>
#include <memory>
#include <per_face_normals.h>
#include <read_triangle_mesh.h>
#include <vector>
#include "GLFW/glfw3.h"
#include "Mesh.h"
#include "PickVisitor.h"
#include "Renderer.h"
#include "IglMeshLoader.h"
#include "igl/per_vertex_normals.h"
#include "igl/per_face_normals.h"
#include "igl/unproject_onto_mesh.h"
#include "igl/edge_flaps.h"
#include "igl/loop.h"
#include "igl/upsample.h"
#include "igl/AABB.h"
#include "igl/parallel_for.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "igl/circulation.h"
#include "igl/edge_midpoints.h"
#include "igl/collapse_edge.h"
#include "igl/edge_collapse_is_valid.h"
#include "igl/write_triangle_mesh.h"
#include <null.cpp>
#include <utility>
#include <cmath>
#include <random>
#include <imgui.h>
#include "CamModel.h"
#include <thread>


using namespace cg3d;
using namespace Eigen;

#define M_PI_2     1.57079632679489661923   // pi/2
void play_sound() {
    PlaySound("C:\\Users\\User\\Desktop\\rocky.wav", NULL, SND_NOSTOP|SND_ASYNC|SND_LOOP);
}
void BasicScene::Init(float fov, int width, int height, float near1, float far1)
{
    std::thread t(play_sound);
    t.join();
    DISPLAY_HEIGHT = height;
    DISPLAY_WIDTH = width;
    generalCamera = Camera::Create("camera", fov, float(width) / height, near1, far1);
    generalCamera->Translate(22, Axis::Z);
    firstPersonCamera = Camera::Create("", fov, double(width) / height, near1, far1);
    camera = generalCamera;
    
    //AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

    auto program = std::make_shared<Program>("shaders/phongShader");
    program->name = "red";
    auto program1 = std::make_shared<Program>("shaders/phongShader");
    program1->name = "blue";
    auto program2 = std::make_shared<Program>("shaders/basicShader");

    material = std::make_shared<Material>("material", program); // empty material
    material1 = std::make_shared<Material>("material1", program1); // empty material
    material1->AddTexture(0, "textures/box0.bmp", 2);
    //material->AddTexture(0, "textures/box0.bmp", 2);
    sphereMesh = IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj");
    cubeMesh = IglLoader::MeshFromFiles("cube_igl", "data/cube_old.obj");
    cylMesh = IglLoader::MeshFromFiles("cyl_igl","data/zcylinder.obj");
    //auto cubeMesh{ IglLoader::MeshFromFiles("cube_igl","data/cube_old.obj") };


    /*adding new roots*/
    // generalRoot = Model::Create("genRoot", cubeMesh, material);
    generalRoot = Movable::Create("generalRoot");
    //generalRoot->isHidden = true;
    
    root = Movable::Create("root");
    //fakeRoot = Movable::Create("root");
    root->AddChild(generalRoot);
    AddChild(root);
    generalRoot->AddChild(camera);
    /*finish adding new roots*/

    //Axis
    Eigen::MatrixXd vertices(6, 3);
    vertices << -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1;
    Eigen::MatrixXi faces(3, 2);
    faces << 0, 1, 2, 3, 4, 5;
    Eigen::MatrixXd vertexNormals = Eigen::MatrixXd::Ones(6, 3);
    Eigen::MatrixXd textureCoords = Eigen::MatrixXd::Ones(6, 2);
    std::shared_ptr<Mesh> coordsys = std::make_shared<Mesh>("coordsys", vertices, faces, vertexNormals, textureCoords);

    sphere = Model::Create("Sphere", sphereMesh, material1);
    //sphere->isHidden = true;
    
    /*Init collision trees for sphere*/
    //MatrixXd V = sphere->GetMesh()->data[0].vertices;
    //MatrixXi F = sphere->GetMesh()->data[0].faces;
    //igl::AABB<MatrixXd, 3> sphereTree;
    //sphereTree.init(V, F);
    //collisionTrees.push_back(sphereTree);
    /*finish init collision trees*/
   /* auto bigSphere = Model::Create("Sphere", sphereMesh, material1);
    
    bigSphere->Scale(1.5f);
    bigSphere->showFaces = true;
    sphere->AddChild(bigSphere);*/
    /*sphere->Scale(scaleFactor, Axis::Z);
    sphere->SetTransform(Eigen::Matrix4f::Identity());
    auto point = GenerateRandomPoint(camera, 5, 35);
    sphere->Translate(point);
    generalRoot->AddChild(sphere);*/
    initAll();
    initObjs(1);
}

void BasicScene::initAll()
{
    for (int i = 0; i < numOfCyl; i++) {
        cyls.push_back(Model::Create("Cyl" + std::to_string(i), cylMesh, material));
        if (i == 0) // first axis and cylinder depend on scene's root
            root->AddChild(cyls[0]);
        else // rest of the axis and cylinders depend on the previous cylinder
            cyls[i - 1]->AddChild(cyls[i]);
    }

    //init the first cyl
    cyls[0]->Scale(scaleFactor, Axis::Z);
    cyls[0]->Translate({ 0, 0, 0.8f * scaleFactor });
    cyls[0]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
    cyls[0]->name = "first";
    //finish init first cyl

    /*init first cyl AABB*/
    /*Init collision trees for sphere*/
    MatrixXd V = cyls[0]->GetMesh()->data[0].vertices;
    MatrixXi F = cyls[0]->GetMesh()->data[0].faces;
    cylTree.init(V, F);
    /*finish init collision trees*/
    /*finish init first cyl AABB*/
    /*root->AddChild(cyls[0]= CamModel::Create(*camera, *cyls[0]));*/
    cyls[0]->AddChild(firstPersonCamera);
    firstPersonCamera->SetTout(Eigen::Affine3f::Identity());
    firstPersonCamera->Translate(1, Axis::Z);
    firstPersonCamera->RotateByDegree(180, Eigen::Vector3f(1, 0, 0));


    //init other cyls
    for (int i = 1; i < numOfCyl; i++)
    {
        cyls[i]->Scale(scaleFactor, Axis::Z);
        cyls[i]->Translate(1.6f * scaleFactor, Axis::Z);
        cyls[i]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
    }
    //finish init other cyls

    //cyls[0]->Rotate((float)(-M_PI_2), cg3d::Movable::Axis::X);

    // move sphere to initial position
    

    cylToMove = (int)cyls.size() - 1;
}

Eigen::Vector3f BasicScene::getSpherePosition()
{
    return sphere->GetAggregatedTransform().col(3).head(3);
}

bool BasicScene::isReachable()
{
    auto spherePos = getSpherePosition();
    auto rootOfArmPos = getEndpoint(0, {0,0,-0.8f}, cyls);
    float dist = std::abs((spherePos - rootOfArmPos).norm());
    float maxArmLength = numOfCyl * 1.6f;
    float maxReachableDistance = maxArmLength * scaleFactor + 0.5f;
    return dist <= maxReachableDistance;
}

Eigen::Vector3f BasicScene::GenerateRandomPoint(std::shared_ptr<cg3d::Camera> camera, float near1, float far1) {
    // Define a random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    // Generate a random point within the frustum defined by the camera's field of view
    float z = std::uniform_real_distribution<>(near1, far1)(gen);
    float x = tan((camera->fov * (M_PI / 180)) / 2) * z * camera->ratio;
    float y = tan((camera->fov * (M_PI / 180)) / 2) * z;

    // Generate random x and y values proportional to the tangent of half the field of view and the distance from the camera to the point
    std::uniform_real_distribution<> dist1(-x, x);
    x = dist1(gen);
    std::uniform_real_distribution<> dist2(-y, y);
    y = dist2(gen);


    // Transform the point from camera space to world space
    Eigen::Vector3f result = camera->GetTranslation() + camera->GetRotation() * Eigen::Vector3f(x, y, -z);
    return result;
}






















void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    if (strcmp(program.name.c_str(), "red") == 0)
    {
        program.SetUniform4f("lightColor", 0.8f, 0.3f, 0.0f, 0.5f);
        program.SetUniform4f("Kai", 1.0f, 0.3f, 0.6f, 1.0f);
        program.SetUniform4f("Kdi", 0.5f, 0.5f, 0.0f, 1.0f);
        program.SetUniform1f("specular_exponent", 5.0f);
        program.SetUniform4f("light_position", 0.0, 15.0f, 0.0, 1.0f);
    }
    else if (strcmp(program.name.c_str(), "blue") == 0) {
        program.SetUniform4f("lightColor", 0.0f, 0.3f, 0.6f, 1.0f);
        program.SetUniform4f("Kai", 0.0f, 0.5f, 1.0f, 1.0f);
        program.SetUniform4f("Kdi", 0.2f, 0.2f, 0.5f, 1.0f);
        program.SetUniform1f("specular_exponent", 10.0f);
        program.SetUniform4f("light_position", 0.0, 15.0f, 0.0, 1.0f);
    }

}

void BasicScene::KeyCallback(Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods)
{
    Matrix3f system = firstPersonCamera->GetRotation().transpose();
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        switch (key) {
        case GLFW_KEY_SPACE:
            animate = true;
            break;
        case GLFW_KEY_LEFT:
        {
            /*cyls[numOfCyl - 1]->TranslateInSystem(system, {-1, 0, 0});
            Vector3f end = getEndpoint(numOfCyl - 1, Vector3f(0, 0, 0.8f), cyls);
            cyls[numOfCyl - 1]->TranslateInSystem(system, { 1, 0, 0});
            snakeFabrik(cyls, end);*/

            cyls[0]->RotateInSystem(system, 0.1f, Axis::Y);
            //cyls[2]->RotateInSystem(system, -0.1f, Axis::Z);
        }
        break;
        case GLFW_KEY_RIGHT:
        {
            /*cyls[numOfCyl - 1]->TranslateInSystem(system, { 1, 0, 0 });
            Vector3f end = getEndpoint(numOfCyl - 1, Vector3f(0, 0, 0.8f), cyls);
            cyls[numOfCyl - 1]->TranslateInSystem(system, { -1, 0, 0 });
            snakeFabrik(cyls, end);*/
            cyls[0]->RotateInSystem(system, -0.1f, Axis::Y);
            //cyls[2]->RotateInSystem(system, 0.1f, Axis::Z);
        }
        break;

        case GLFW_KEY_UP:
        {
            /*cyls[numOfCyl - 1]->TranslateInSystem(system, {0, 1, 0});
            Vector3f end = getEndpoint(numOfCyl - 1, Vector3f(0, 0, 0.8f), cyls);
            cyls[numOfCyl - 1]->TranslateInSystem(system, { 0, -1, 0 });
            snakeFabrik(cyls, end);*/
            cyls[0]->RotateInSystem(system, 0.1f, Axis::X);
            //cyls[2]->RotateInSystem(system, 0.1f, Axis::X);

        }
        break;
        case GLFW_KEY_DOWN:
        {
            /*cyls[numOfCyl - 1]->TranslateInSystem(system, { 0, -1, 0 });
            Vector3f end = getEndpoint(numOfCyl - 1, Vector3f(0, 0, 0.8f), cyls);
            cyls[numOfCyl - 1]->TranslateInSystem(system, { 0, 1, 0 });
            snakeFabrik(cyls, end);
            */
            cyls[0]->RotateInSystem(system, -0.1f, Axis::X);
            //cyls[2]->RotateInSystem(system, -0.1, Axis::X);

        }
        break;
        default:
            break;
        }
    }

    //SceneWithImGui::KeyCallback(nullptr, x, y, key, scancode, action, mods);
}

void BasicScene::AddViewportCallback(cg3d::Viewport* _viewport)
{
    viewport = _viewport;

    Scene::AddViewportCallback(viewport);
}

void BasicScene::initObjs(int level)
{
    for (int i = 0; i < 3 + level; i++)
    {
        std::shared_ptr<cg3d::Model> mod = Model::Create("Sphere", sphereMesh, material1);
        root->AddChild(mod);
        MatrixXd V = mod->GetMesh()->data[0].vertices;
        MatrixXi F = mod->GetMesh()->data[0].faces;
        igl::AABB<MatrixXd, 3> modelTree;
        modelTree.init(V, F);

        objs.push_back(mod);
        collisionTrees.push_back(modelTree);
    }

    for (int i = 0; i < level * 2; i++)
    {
        std::shared_ptr<cg3d::Model> mod = Model::Create("Cube", cubeMesh, material1);
        root->AddChild(mod);
        MatrixXd V = mod->GetMesh()->data[0].vertices;
        MatrixXi F = mod->GetMesh()->data[0].faces;
        igl::AABB<MatrixXd, 3> modelTree;
        modelTree.init(V, F);

        objs.push_back(mod);
        collisionTrees.push_back(modelTree);
    }
    
    for (int i = 0; i < objs.size(); i++) {
        auto newPoint = GenerateRandomPoint(camera, 5, 35);
        objs[i]->SetTransform(Eigen::Matrix4f::Identity());
        objs[i]->Translate(newPoint);
    }
}



void BasicScene::MouseCallback(Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[])
{
    // note: there's a (small) chance the button state here precedes the mouse press/release event
    //if (ImGui::GetIO().WantCaptureMouse)
    //    return;
    //if (action == GLFW_PRESS) { // default mouse button press behavior
    //    PickVisitor visitor;
    //    visitor.Init();
    //    renderer->RenderViewportAtPos(x, y, &visitor); // pick using fixed colors hack
    //    auto modelAndDepth = visitor.PickAtPos(x, renderer->GetWindowHeight() - y);
    //    renderer->RenderViewportAtPos(x, y); // draw again to avoid flickering
    //    pickedModel = modelAndDepth.first ? std::dynamic_pointer_cast<Model>(modelAndDepth.first->shared_from_this()) : nullptr;
    //    pickedModelDepth = modelAndDepth.second;
    //    camera->GetRotation().transpose();
    //    xAtPress = x;
    //    yAtPress = y;
    //    lastx = x;
    //    lasty = y;
    //    if (pickedModel && !pickedModel->isPickable)
    //        pickedModel = nullptr; // for non-pickable models we need only pickedModelDepth for mouse movement calculations later

    //    if (pickedModel)
    //        pickedToutAtPress = pickedModel->GetTout();
    //    else
    //        cameraToutAtPress = camera->GetTout();
    //}
    if (ImGui::GetIO().WantCaptureMouse)
        return;
    if (action == GLFW_PRESS)
    { // default mouse button press behavior
        PickVisitor visitor;
        visitor.Init();
        renderer->RenderViewportAtPos(x, y, &visitor); // pick using fixed colors hack
        auto modelAndDepth = visitor.PickAtPos(x, renderer->GetWindowHeight() - y);
        renderer->RenderViewportAtPos(x, y); // draw again to avoid flickering
        pickedModel = modelAndDepth.first ? std::dynamic_pointer_cast<Model>(modelAndDepth.first->shared_from_this()) : nullptr;
        // pickedModel = pickedModel->modelOnPick != nullptr ? pickedModel->modelOnPick : pickedModel;
        pickedModelDepth = modelAndDepth.second;
        camera->GetRotation().transpose();
        xAtPress = x;
        yAtPress = y;
        lastx = x;
        lasty = y;

        if (pickedModel && !pickedModel->isPickable)
            pickedModel = nullptr; // for non-pickable models we need only pickedModelDepth for mouse movement calculations later

        if (pickedModel)
        {
            pickedToutAtPress = pickedModel->GetTout();
            otherPickedToutAtPress = pickedModel != nullptr ? pickedModel->GetTout() : pickedModel->GetTout();
        }

        else
            cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::ScrollCallback(Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[])
{
    //if (ImGui::GetIO().WantCaptureMouse)
    //    return;
    //// note: there's a (small) chance the button state here precedes the mouse press/release event
    //auto system = camera->GetRotation().transpose();
    //if (pickedModel) {
    //    pickedModel->TranslateInSystem(system, { 0, 0, -float(yoffset) });
    //    pickedToutAtPress = pickedModel->GetTout();
    //}
    //else {
    //    camera->TranslateInSystem(system, { 0, 0, -float(yoffset) });
    //    cameraToutAtPress = camera->GetTout();
    //}
    if (ImGui::GetIO().WantCaptureMouse)
        return;
    // note: there's a (small) chance the button state here precedes the mouse press/release event
    auto system = camera->GetRotation().transpose();
    if (pickedModel)
    {
        if (pickedModel != nullptr)
        {
            pickedModel->TranslateInSystem(system, { 0, 0, -float(yoffset) });
            otherPickedToutAtPress = pickedModel->GetTout();
        }
        else
        {
            pickedModel->TranslateInSystem(system, { 0, 0, -float(yoffset) });
            pickedToutAtPress = pickedModel->GetTout();
        }
    }
    else
    {
        camera->TranslateInSystem(system, { 0, 0, -float(yoffset) });
        cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::CursorPosCallback(Viewport* viewport, int x, int y, bool dragging, int* buttonState)
{
    //if (ImGui::GetIO().WantCaptureMouse)
    //    return;
    //if (dragging) {
    //    auto system = camera->GetRotation().transpose() * GetRotation();
    //    auto moveCoeff = camera->CalcMoveCoeff(pickedModelDepth, viewport->width);
    //    auto angleCoeff = camera->CalcAngleCoeff(viewport->width);
    //    if (pickedModel) {
    //        //pickedModel->SetTout(pickedToutAtPress);
    //        if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
    //            if (pickedModel->name == "cyl") {
    //                cyls[0]->TranslateInSystem(system, { -float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });
    //            }
    //            else {
    //                pickedModel->TranslateInSystem(system, { -float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });
    //            }
    //        if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
    //            pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Z);
    //        if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
    //            pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Y);
    //            pickedModel->RotateInSystem(system, float(yAtPress - y) / angleCoeff, Axis::X);
    //        }
    //    }
    //    else {
    //        /*camera->SetTout(cameraToutAtPress);
    //        if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
    //            camera->TranslateInSystem(system, { -float(xAtPress - x) / moveCoeff / 10.0f, float(yAtPress - y) / moveCoeff / 10.0f, 0 });
    //        if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
    //            camera->RotateInSystem(system, float(x - xAtPress) / 180.0f, Axis::Z);
    //        if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
    //            camera->RotateInSystem(system, float(x - xAtPress) / angleCoeff, Axis::Y);
    //            camera->RotateInSystem(system, float(y - yAtPress) / angleCoeff, Axis::X);
    //        }*/
    //        camera->SetTout(cameraToutAtPress);
    //        if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE)
    //        {
    //            root->RotateInSystem(system, float(x - lastx) / angleCoeff * 3, Axis::Y);
    //            root->RotateInSystem(system, float(y - lasty) / angleCoeff * 3, Axis::X);
    //            lastx = x;
    //            lasty = y;
    //        }
    //        if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
    //            camera->RotateInSystem(system, float(x - xAtPress) / 180, Axis::Z);
    //        if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE) {
    //            root->TranslateInSystem(system, { float(lastx - x) / moveCoeff * 0.2f, float(y - lasty) / moveCoeff * 0.2f, 0 });
    //            lastx = x;
    //            lasty = y;
    //        }
    //    }
    //    xAtPress = x;
    //    yAtPress = y;
    //}
    if (ImGui::GetIO().WantCaptureMouse)
        return;
    std::shared_ptr<cg3d::Model> actuallyPicked = pickedModel;
    // std::cout << "before dragging" << std::endl;
    if (dragging)
    {
        auto system = camera->GetRotation().transpose();
        auto moveCoeff = camera->CalcMoveCoeff(pickedModelDepth, viewport->width);
        auto angleCoeff = camera->CalcAngleCoeff(viewport->width);
        if (pickedModel)
        {
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE)
            {
                pickedModel->SetTout(pickedToutAtPress);
                pickedModel->RotateInSystem(system, float(x - xAtPress) / moveCoeff, Axis::Y);
                pickedModel->RotateInSystem(system, float(y - yAtPress) / moveCoeff, Axis::X);
            }
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
            {
                pickedModel->SetTout(pickedToutAtPress);
                pickedModel->RotateInSystem(system, float(x - xAtPress) / moveCoeff, Axis::Z);
            }
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
            {
                if (pickedModel != nullptr)
                {
                    actuallyPicked = pickedModel;
                    pickedModel->SetTout(otherPickedToutAtPress);
                }
                pickedModel->SetTout(pickedToutAtPress);
                actuallyPicked->TranslateInSystem(system * generalRoot->GetRotation(), { float(x - xAtPress) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });
            }
        }
        else
        {
            camera->SetTout(cameraToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE)
            {
                generalRoot->RotateInSystem(system, float(x - lastx) / angleCoeff * 3, Axis::Y);
                generalRoot->RotateInSystem(system, float(y - lasty) / angleCoeff * 3, Axis::X);
                lastx = x;
                lasty = y;
            }
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
            {
                camera->RotateInSystem(system, float(x - xAtPress) / 180, Axis::Z);
            }
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
            {
                generalRoot->TranslateInSystem(system, { float(lastx - x) / moveCoeff * 0.2f, float(y - lasty) / moveCoeff * 0.2f, 0 });
                lastx = x;
                lasty = y;
            }
        }
    }
}

void TextCentered(std::string text, float margin = 0)
{
    auto windowWidth = ImGui::GetWindowSize().x;
    auto textWidth = ImGui::CalcTextSize(text.c_str()).x;

    ImGui::SetCursorPosX((windowWidth - textWidth) * 0.5f);
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + margin);

    ImGui::Text(text.c_str());
}


void cursorCentered(std::string text, float margin = 0)
{
    auto windowWidth = ImGui::GetWindowSize().x;
    auto textWidth = ImGui::CalcTextSize(text.c_str()).x;

    ImGui::SetCursorPosX((windowWidth - textWidth) * 0.5f);
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + margin);
}

void BasicScene::BuildImGui()
{
    int flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
    bool* pOpen = nullptr;
    ImGui::Begin("Menu", pOpen, flags);

    switch (gameMode)
    {
    case gameMode::StartMenu:
    {

        float window_width = 200;
        float window_height = 190;
        ImGui::SetWindowPos(ImVec2((DISPLAY_WIDTH - window_width) / 2, (DISPLAY_HEIGHT - window_height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(window_width, window_height));

        TextCentered("Snake 3D", 30);
        cursorCentered("Start Game", 25);
        if (ImGui::Button("Start Game"))
        {
            gameMode = gameMode::Start;
        }
        cursorCentered("Quit Game", 25);
        if (ImGui::Button("Quit Game"))
        {
            
            exit(0);
        }
        break;
    }
    case gameMode::Start:
    {
        float window_width = 400;
        float window_height = 400;
        ImGui::SetWindowPos(ImVec2(0, 0), ImGuiCond_Always);

        ImGui::SetWindowSize(ImVec2(window_width, window_height));
        TextCentered("Welcome to LVL " + std::to_string(currLevel), 10);
        if (nextLevelFlag) {
            gameMode = gameMode::Choose;
            animate = false;
        }
        
        TextCentered("Every 10 Point, You Move To The Next Level", 10);
        TextCentered("Controls:", 10);
        TextCentered("Move Using Arrows", 10);
        TextCentered("Space: Stop/Play", 10);
        TextCentered("Q: Quits Game", 10);
        TextCentered("-------------- Score:" + std::to_string(score) + "  --------------", 10);
        TextCentered("-------------- Lives:" + std::to_string(lives) + "  --------------", 10);

        cursorCentered("Start Playing", 10);
        if (ImGui::Button("Start Playing"))
            animate = true;

        cursorCentered("Reset", 10);
        if (ImGui::Button("Reset")) {
            animate = false;
            score = 0;
            speed = 0.01f;
            cyls[0]->SetTransform(Matrix4f::Identity());
            cyls[0]->Translate({ 0,0,0 });
            camera = generalCamera;
            viewport->camera = camera;
        }

        cursorCentered("Main Camera", 10);
        if (ImGui::Button("Main Camera")) {
            camera = generalCamera;
            viewport->camera = camera;
        }

        cursorCentered("First Person Camera", 10);
        if (ImGui::Button("First Person Camera")) {
            camera = firstPersonCamera;
            viewport->camera = camera;
        }

        cursorCentered("Quit Game", 10);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;

    }
    case gameMode::Choose:
    {
        animate = false;
        float window_width = 300;
        float window_height = 200;
        ImGui::SetWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(window_width, window_height));
        TextCentered("Congratulations!!!!!!!", 30);
        TextCentered("You Finished Level " + std::to_string(currLevel), 30);
        cursorCentered("Start Level Again", 10);
        if (ImGui::Button("Start Level Again")) {
            gameMode = gameMode::Start;
            ResetToLevel(currLevel, 3);
            nextLevelFlag = false;
        }

        cursorCentered("Start Next Level", 10);
        if (ImGui::Button("Start Next Level")) {
            gameMode = gameMode::Start;
            currLevel++;
            ResetToLevel(currLevel, lives);
            nextLevelFlag = false;
        }
        break;
    }

    default:
    {
        break;
    }
    }

    ImGui::End();
}

void BasicScene::ResetToLevel(int level, int lives) {
    //re-arange objects = reset objects, reset snake
    //if lives == 0: restart all game (including reset menu)
    //else, only reset to current level, with same stats
    if (lives == 0) {
        score = 0;
        currLevel = 1;
        speed = 0.01f;
        reArangeObjects(currLevel);
    }
    else {
        reArangeObjects(level);
    }
}

void BasicScene::reArangeObjects(int level) {
    cyls[0]->SetTransform(Matrix4f::Identity());
    for (auto obj : objs) {
        root->RemoveChild(obj);
    }
    objs.clear();
    collisionTrees.clear();
    initObjs(level);
    camera = generalCamera;
    viewport->camera = camera;
}

