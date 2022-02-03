#include "TutorialGame.h"
#include "../CSC8503Common/GameWorld.h"
#include "../../Plugins/OpenGLRendering/OGLMesh.h"
#include "../../Plugins/OpenGLRendering/OGLShader.h"
#include "../../Plugins/OpenGLRendering/OGLTexture.h"
#include "../../Common/TextureLoader.h"
#include "../CSC8503Common/NavigationGrid.h"
#include <fstream>
#include "../../Common/Assets.h"
#include "../CSC8503Common/State.h"
#include "../CSC8503Common/StateTransition.h"
#include "../CSC8503Common/StateMachine.h"
#include "../CSC8503Common/BehaviourAction.h"
#include "../CSC8503Common/BehaviourSequence.h"
#include "../CSC8503Common/BehaviourSelector.h"
using namespace std;
using namespace NCL::CSC8503;

using namespace NCL;
using namespace CSC8503;

TutorialGame::TutorialGame()	{
	world		= new GameWorld();
	renderer	= new GameTechRenderer(*world);
	physics		= new PhysicsSystem(*world);

	forceMagnitude	= 10.0f;
	useGravity		= false;
	inSelectionMode = false;

	Debug::SetRenderer(renderer);

	InitialiseAssets();
}
void TutorialGame::Paused(float dt) {
	renderer->DrawString("Game Paused Press U to unpause", Vector2(20,50));
	renderer->Update(dt);
	Debug::FlushRenderables(dt);
	renderer->Render();
}
void TutorialGame::End(float dt) {
	if (score > 0) {
		renderer->DrawString("Game Won", Vector2(20, 40));
	}
	else {
		renderer->DrawString("Game Lost", Vector2(20, 40));
	}
	renderer->DrawString("Time Taken: " + gameTime, Vector2(20, 50));
	renderer->DrawString("Score: " + to_string(score), Vector2(20, 60));

	renderer->DrawString("Press F1 to return to main menu", Vector2(20, 80));	
	renderer->Update(dt);
	Debug::FlushRenderables(dt);
	renderer->Render();
	endScreenShown = true;
	gameCompleted = false;
}
/*

Each of the little demo scenarios used in the game uses the same 2 meshes, 
and the same texture and shader. There's no need to ever load in anything else
for this module, even in the coursework, but you can add it if you like!

*/
void TutorialGame::InitialiseAssets() {
	auto loadFunc = [](const string& name, OGLMesh** into) {
		*into = new OGLMesh(name);
		(*into)->SetPrimitiveType(GeometryPrimitive::Triangles);
		(*into)->UploadToGPU();
	};

	loadFunc("cube.msh"		 , &cubeMesh);
	loadFunc("sphere.msh"	 , &sphereMesh);
	loadFunc("Male1.msh"	 , &charMeshA);
	loadFunc("courier.msh"	 , &charMeshB);
	loadFunc("security.msh"	 , &enemyMesh);
	loadFunc("coin.msh"		 , &bonusMesh);
	loadFunc("capsule.msh"	 , &capsuleMesh);

	basicTex	= (OGLTexture*)TextureLoader::LoadAPITexture("checkerboard.png");
	basicShader = new OGLShader("GameTechVert.glsl", "GameTechFrag.glsl");

	InitCamera();
	//InitWorld();
	if (initialise == true && gameType == 1) {
		InitMap();
		startTime = chrono::high_resolution_clock::now();
		score = 0;
	}

	if (initialise == true && gameType == 2) {
		DrawMaze();
		aimPosition = Vector3(210,200,10);
		TestBehaviourTree();
		startTime = chrono::high_resolution_clock::now();
		score = 0;
	}
	
}

void TutorialGame::DrawMaze() {
	string filename = "TestGrid1.txt";
	ifstream infile(Assets::DATADIR + filename);
	string temp;
	int j = 0;
	for (temp; getline(infile, temp);) {
		if (temp[0] == 'x') {
			int i = 0;
			for (auto t : temp) {
				if (t == 'x') {
					AddCubeToWorld(Vector3(10 * i, 200, 10 * j), Vector3(5, 5, 5), 0.0f,Vector4(0,1,0,1));
				}
				else {
					int placePowerUp = rand() % 40;
					if (placePowerUp == 3) {
						GameObject* powerup = AddBonusToWorld(Vector3(10 * i, 201, 10 * j));
						powerup->setTrigger(true);
						powerups.emplace_back(powerup);

					}
				}
				i++;
			}
			j++;
		}
		
	}
	infile.close();
	AddCubeToWorld(Vector3(125, 190, 125),Vector3(120,5,120),0.0f,Vector4(1,0,0,1));
	playerBall = AddSphereToWorld(Vector3(10, 200, 10), 2.0f, 5.0f);
	enemyBall = AddSphereToWorld(Vector3(230, 200, 10), 2.0f, 5.0f);


}

Vector3 TutorialGame::TestPathFinding() {
	vector<Vector3>testNodes = PathFinding(target);
	for (int i = 1; i < testNodes.size(); ++i) {
		Vector3 a = testNodes[i - 1];
		Vector3 b = testNodes[i];
		Debug::DrawLine(a, b, Vector4(1, 0, 0, 1));
	}
	if (testNodes.size() > 0) {
		return testNodes[1];
	}
	else {
		return testNodes[0];
	}
}
std::vector<Vector3> TutorialGame::PathFinding(GameObject* target) {
	vector<Vector3>testNodes;
	NavigationGrid grid("TestGrid1.txt");
	NavigationPath outPath;
	Vector3 startPos(enemyBall->GetTransform().GetPosition());
	Vector3 endPos(target->GetTransform().GetPosition());
	bool found = grid.FindPath(startPos, endPos, outPath);
	Vector3 pos;
	while (outPath.PopWaypoint(pos)) {
		testNodes.push_back(pos);
	}
	return testNodes;
}
void TutorialGame::MoveEnemy() {
	if (abs((aimPosition - enemyBall->GetTransform().GetPosition()).x) < 2 && abs((aimPosition - enemyBall->GetTransform().GetPosition()).z) < 2) {
		aimPosition = TestPathFinding();
	}
	else {
		enemyBall->GetTransform().SetPosition(enemyBall->GetTransform().GetPosition() + (aimPosition - enemyBall->GetTransform().GetPosition()).Normalised() /2 );
	}
}
GameObject* TutorialGame::FindClosestPowerUp() {
	float distance = FLT_MAX;
	GameObject* closestNode = new GameObject();
	for (int i = 0; i < powerups.size(); i++) {
		std::vector<Vector3>testNodes = PathFinding(powerups[i]);
		if (testNodes.size() < distance) {
			distance = testNodes.size();
			closestNode = powerups[i];
		}
	}
	return closestNode;
}
void TutorialGame::TestBehaviourTree() {
	BehaviourAction* findPowerUp = new BehaviourAction("Find PowerUp", [&](float dt, BehaviourState state)->BehaviourState {
		if (state == Initialise) {
			target = FindClosestPowerUp();
			if (powerups.size() > 0) {
				state = Ongoing;
			}
		}
		else if (state == Ongoing) {

			if ((abs((target->GetTransform().GetPosition() - enemyBall->GetTransform().GetPosition()).x) < 10 && abs((target->GetTransform().GetPosition() - enemyBall->GetTransform().GetPosition()).z) < 10)) {
				world->RemoveGameObject(target);
				powerups.erase(std::remove(powerups.begin(), powerups.end(), target), powerups.end());
				score -= 50;
				return Success;
			}
			if (PathFinding(playerBall).size() < 10) {
				return Failure;
			}
			
		}
		return state;
		}
	);
	BehaviourAction* findPlayer = new BehaviourAction("Find PowerUp", [&](float dt, BehaviourState state)->BehaviourState {
		if (state == Initialise) {
			target = playerBall;
			state = Ongoing;
		}
		else if (state == Ongoing) {
			if (PathFinding(target).size() >= 10) {
				return Failure;
			}
			if ((abs((target->GetTransform().GetPosition() - enemyBall->GetTransform().GetPosition()).x) < 10 && abs((target->GetTransform().GetPosition() - enemyBall->GetTransform().GetPosition()).z) < 10)) {
				playerBall->GetTransform().SetPosition(Vector3(10, 200, 10));
				enemyBall->GetTransform().SetPosition(Vector3(230, 200, 10));
				score -= 100;
				return Success;
			}
			
		}
		return state;
		}
	);
	selection->AddChild(findPowerUp);
	selection->AddChild(findPlayer);
}
TutorialGame::~TutorialGame()	{
	delete cubeMesh;
	delete sphereMesh;
	delete charMeshA;
	delete charMeshB;
	delete enemyMesh;
	delete bonusMesh;

	delete basicTex;
	delete basicShader;

	delete physics;
	delete renderer;
	delete world;
}

void TutorialGame::UpdateGame(float dt) {
	if (!inSelectionMode) {
		world->GetMainCamera()->UpdateCamera(dt);
	}

	UpdateKeys();
	if (initialise) {


		if (useGravity) {
			Debug::Print("(G)ravity on", Vector2(5, 95));
		}
		else {
			Debug::Print("(G)ravity off", Vector2(5, 95));
		}
		endTime = chrono::high_resolution_clock::now();
		chrono::duration<double> time = chrono::duration_cast<chrono::duration<double>>(endTime - startTime);
		auto minutes = chrono::floor<chrono::minutes>(time).count();
		auto seconds = chrono::floor<chrono::seconds>(time).count() % 60;
		gameTime = to_string(minutes) + "m" + to_string(seconds) + "s";
		renderer->DrawString("Game Time: " + gameTime, Vector2(70, 95));
		renderer->DrawString("Score: " + to_string(score), Vector2(70, 90));
	}

	if (testStateObject) {
		testStateObject->Update(dt);
	}
	if (initialise) {
		SelectObject();
	}
	MoveSelectedObject();

	physics->Update(dt);
	AddRopeToBall();
	if (lockedObject != nullptr) {
		Vector3 objPos = lockedObject->GetTransform().GetPosition();
		Vector3 camPos = objPos + lockedOffset;

		Matrix4 temp = Matrix4::BuildViewMatrix(camPos, objPos, Vector3(0, 1, 0));

		Matrix4 modelMat = temp.Inverse();

		Quaternion q(modelMat);
		Vector3 angles = q.ToEuler(); //nearly there now!

		world->GetMainCamera()->SetPosition(camPos);
		world->GetMainCamera()->SetPitch(angles.x);
		world->GetMainCamera()->SetYaw(angles.y);

		//Debug::DrawAxisLines(lockedObject->GetTransform().GetMatrix(), 2.0f);
	}
	if (initialise == true && gameType == 2) {

		if (Window::GetKeyboard()->KeyHeld(KeyboardKeys::LEFT)) {
			playerBall->GetTransform().SetPosition(Vector3(playerBall->GetTransform().GetPosition().x, playerBall->GetTransform().GetPosition().y, playerBall->GetTransform().GetPosition().z - 0.5));
		}
		if (Window::GetKeyboard()->KeyHeld(KeyboardKeys::RIGHT)) {
			playerBall->GetTransform().SetPosition(Vector3(playerBall->GetTransform().GetPosition().x, playerBall->GetTransform().GetPosition().y, playerBall->GetTransform().GetPosition().z + 0.5));
		}
		if (Window::GetKeyboard()->KeyHeld(KeyboardKeys::UP)) {
			playerBall->GetTransform().SetPosition(Vector3(playerBall->GetTransform().GetPosition().x + 0.5, playerBall->GetTransform().GetPosition().y, playerBall->GetTransform().GetPosition().z));
		}
		if (Window::GetKeyboard()->KeyHeld(KeyboardKeys::DOWN)) {
			playerBall->GetTransform().SetPosition(Vector3(playerBall->GetTransform().GetPosition().x - 0.5, playerBall->GetTransform().GetPosition().y, playerBall->GetTransform().GetPosition().z));
		}
	}

	if (initialise == true && gameType == 1) {
		CollisionDetection::CollisionInfo info;
		if (capsule && Ball) {
			if ((capsule->GetWorldID() > 0 && Ball->GetWorldID() > 0)) {
				if (CollisionDetection::ObjectIntersection(Ball, capsule, info)) {
					world->RemoveGameObject(capsule);
					capsule = nullptr;
					gameCompleted = true;
				}
			}

		}
		if (checkpoints.size() > 0) {
			reachCheckpoint();
		}
		if (Ball->GetTransform().GetPosition().x < -100 || Ball->GetTransform().GetPosition().x > 100 || Ball->GetTransform().GetPosition().z < -100 || Ball->GetTransform().GetPosition().z > 100) {
			Ball->GetTransform().SetPosition(lastCheckpoint);
		}
	}
	if (initialise == true && gameType == 2) {
		if (powerups.size() > 0) {


			CollectPowerUp();
			if (state == Ongoing) {
				state = selection->Execute(1.0f);
				MoveEnemy();
			}
			if (state == Success) {
				selection->Reset();
				state = Initialise;
				aimPosition = TestPathFinding();
				state = selection->Execute(1.0f);
			}
			if (state == Failure) {
				selection->Reset();
				state = Initialise;
				state = selection->Execute(1.0f);
			}
		}
		else {
			gameCompleted = true;
		}


	}
	
	world->UpdateWorld(dt);
	renderer->Update(dt);

	Debug::FlushRenderables(dt);
	renderer->Render();
}
void TutorialGame::reachCheckpoint() {
	if (checkpoints[0]) {
		if (checkpoints[0]->GetWorldID() > 0) {
			for (int i = 0; i < checkpoints.size(); i++) {
				CollisionDetection::CollisionInfo info;
				if (CollisionDetection::ObjectIntersection(Ball, checkpoints[i], info)) {
					lastCheckpoint = checkpoints[i]->GetTransform().GetPosition();
					world->RemoveGameObject(checkpoints[i]);
					checkpoints.erase(std::remove(checkpoints.begin(), checkpoints.end(), checkpoints[i]), checkpoints.end());
					score += 100;
				}
			}
		}
	}
}
void TutorialGame::CollectPowerUp() {
	if (powerups[0] != nullptr) {
		if (powerups[0]->GetWorldID() > 0) {
			for (int i = 0; i < powerups.size(); i++) {
				CollisionDetection::CollisionInfo info;
				if (CollisionDetection::ObjectIntersection(playerBall, powerups[i], info)) {
					world->RemoveGameObject(powerups[i]);
					powerups.erase(std::remove(powerups.begin(), powerups.end(), powerups[i]), powerups.end());
					score += 100;
				}
			}
		}
		
	}
	
}
void TutorialGame::UpdateKeys() {
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F1)) {
		InitWorld(); //We can reset the simulation at any time with F1
		selectionObject = nullptr;
		lockedObject	= nullptr;
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F2)) {
		InitCamera(); //F2 will reset the camera to a specific default place
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::G)) {
		useGravity = !useGravity; //Toggle gravity!
		physics->UseGravity(useGravity);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::R) && gameType == 1 ) {
		Ball->GetTransform().SetPosition(lastCheckpoint);
	}
	//Running certain physics updates in a consistent order might cause some
	//bias in the calculations - the same objects might keep 'winning' the constraint
	//allowing the other one to stretch too much etc. Shuffling the order so that it
	//is random every frame can help reduce such bias.
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F9)) {
		world->ShuffleConstraints(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F10)) {
		world->ShuffleConstraints(false);
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F7)) {
		world->ShuffleObjects(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F8)) {
		world->ShuffleObjects(false);
	}

	if (lockedObject) {
		LockedObjectMovement();
	}
	else {
		DebugObjectMovement();
	}
}

void TutorialGame::LockedObjectMovement() {
	Matrix4 view		= world->GetMainCamera()->BuildViewMatrix();
	Matrix4 camWorld	= view.Inverse();

	Vector3 rightAxis = Vector3(camWorld.GetColumn(0)); //view is inverse of model!

	//forward is more tricky -  camera forward is 'into' the screen...
	//so we can take a guess, and use the cross of straight up, and
	//the right axis, to hopefully get a vector that's good enough!

	Vector3 fwdAxis = Vector3::Cross(Vector3(0, 1, 0), rightAxis);
	fwdAxis.y = 0.0f;
	fwdAxis.Normalise();

	Vector3 charForward  = lockedObject->GetTransform().GetOrientation() * Vector3(0, 0, 1);
	Vector3 charForward2 = lockedObject->GetTransform().GetOrientation() * Vector3(0, 0, 1);

	float force = 100.0f;

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::LEFT)) {
		lockedObject->GetPhysicsObject()->AddForce(-rightAxis * force);
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
		Vector3 worldPos = selectionObject->GetTransform().GetPosition();
		lockedObject->GetPhysicsObject()->AddForce(rightAxis * force);
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::UP)) {
		lockedObject->GetPhysicsObject()->AddForce(fwdAxis * force);
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::DOWN)) {
		lockedObject->GetPhysicsObject()->AddForce(-fwdAxis * force);
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NEXT)) {
		lockedObject->GetPhysicsObject()->AddForce(Vector3(0,-10,0));
	}
}

void TutorialGame::DebugObjectMovement() {
//If we've selected an object, we can manipulate it with some key presses
	if (inSelectionMode && selectionObject) {
		//Twist the selected object!
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::LEFT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(-10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM7)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, 10, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM8)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, -10, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::UP)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, -10));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::DOWN)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, 10));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM5)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, -10, 0));
		}
	}

}

void TutorialGame::InitCamera() {
	if (gameType == 1) {
		world->GetMainCamera()->SetNearPlane(0.1f);
		world->GetMainCamera()->SetFarPlane(500.0f);
		world->GetMainCamera()->SetPitch(-40.0f);
		world->GetMainCamera()->SetYaw(315.0f);
		world->GetMainCamera()->SetPosition(Vector3(-120, 150, 120));
	}
	else if (gameType == 2) {
		world->GetMainCamera()->SetNearPlane(0.1f);
		world->GetMainCamera()->SetFarPlane(500.0f);
		world->GetMainCamera()->SetPitch(-90.0f);
		world->GetMainCamera()->SetYaw(-90.0f);
		world->GetMainCamera()->SetPosition(Vector3(125, 500, 125));
	}
	else {
		world->GetMainCamera()->SetNearPlane(0.1f);
		world->GetMainCamera()->SetFarPlane(500.0f);
		world->GetMainCamera()->SetPitch(-15.0f);
		world->GetMainCamera()->SetYaw(315.0f);
		world->GetMainCamera()->SetPosition(Vector3(-60, 200, 60));
	}

	lockedObject = nullptr;
}
void TutorialGame::BridgeConstraintTest() {
	Vector3 cubeSize = Vector3(8, 8, 8);
	float invCubeMass = 5; //how heavy the middle pieces are
	int numLinks = 10;
	float maxDistance = 30; // constraint distance
	float cubeDistance = 20; // distance between links
	Vector3 startPos = Vector3(0, 0, 0);
	GameObject* start = AddCubeToWorld(startPos + Vector3(0, 0, 0), cubeSize, 0);
	GameObject* end = AddCubeToWorld(startPos + Vector3((numLinks + 2) * cubeDistance, 0, 0), cubeSize, 0);
	GameObject* previous = start;
	for (int i = 0; i < numLinks; ++i) {
		GameObject* block = AddCubeToWorld(startPos + Vector3((i + 1) * cubeDistance, 0, 0), cubeSize, invCubeMass);
		PositionConstraint* constraint = new PositionConstraint(previous, block, maxDistance);
		world->AddConstraint(constraint);
		previous = block;
	}
	PositionConstraint* constraint = new PositionConstraint(previous, end, maxDistance);
	world->AddConstraint(constraint);
}

void TutorialGame::InitMap() {
	AddFloorToWorld(Vector3(0, 50, 0));
	AddCubeToWorld(Vector3(-98, 62, 0), Vector3(2, 10, 100), 0.0f);
	AddCubeToWorld(Vector3(98, 62, 0), Vector3(2, 10, 100), 0.0f);
	AddCubeToWorld(Vector3(0, 62, -98), Vector3(96, 10, 2), 0.0f);
	AddCubeToWorld(Vector3(0, 62, 98), Vector3(96, 10, 2), 0.0f);
	AddCubeToWorld(Vector3(-70, 62, 60), Vector3(10, 2, 30), 0.0f, Vector4(1, 0, 0, 1), true, Vector3(-15, 0, 0));
	AddCubeToWorld(Vector3(-70, 62, 0), Vector3(10, 2, 30), 0.0f, Vector4(1, 0, 0, 1), true, Vector3(15, 0, 0));
	Ball = AddSphereToWorld(Vector3(-70, 75, 80), 2.0f);
	Ball->GetPhysicsObject()->setElasticity(0.9);
	AddCubeToWorld(Vector3(-70, 60, -40), Vector3(10, 10, 10), 0.0f, Vector4(0, 1, 0, 1));
	AddCubeToWorld(Vector3(-70, 62, -62), Vector3(10, 12, 10), 0.0f, Vector4(0, 1, 0, 1));
	AddCubeToWorld(Vector3(-70, 64, -84), Vector3(10, 14, 10), 0.0f, Vector4(0, 1, 0, 1));
	AddCubeToWorld(Vector3(-70, 88, -93), Vector3(10, 10, 1), 0.0f);
	AddCubeToWorld(Vector3(-81, 64, -75), Vector3(1, 14, 1), 0.0f);
	AddCubeToWorld(Vector3(-81, 80, -75), Vector3(1, 1, 16), 0.0f, Vector4(0, 0, 1, 1), true);
	AddCubeToWorld(Vector3(-10, 110, -80), Vector3(40, 2, 20), 0.0f, Vector4(1, 1, 1, 1), true, Vector3(0, 0, 70));
	AddCubeToWorld(Vector3(-25, 100, -60), Vector3(20, 2, 40), 0.0f, Vector4(1, 1, 1, 1), true, Vector3(-90, 0, 0));
	AddCubeToWorld(Vector3(-35, 100, -100), Vector3(30, 2, 40), 0.0f, Vector4(1, 1, 1, 1), true, Vector3(90, 0, 0));
	AddCubeToWorld(Vector3(-50, 65, -80), Vector3(10, 2, 20), 0.0f, Vector4(1, 1, 1, 1), true, Vector3(0, 0, -40));
	AddCubeToWorld(Vector3(0, 55, -80), Vector3(50, 2, 20), 0.0f, Vector4(1, 1, 1, 1), true, Vector3(0, 0, -5));
	tether = AddCubeToWorld(Vector3(30, 55, -30), Vector3(1, 10, 1), 0.0f, Vector4(0.5, 1, 0, 1));
	AddCapsuleToWorld(Vector3(0, 20, 0), 5.0f, 2.0f);
	AddCubeToWorld(Vector3(10, 20, 0), Vector3(5, 5, 5), 10.0f, Vector4(1, 1, 1, 1), true);
	AddCubeToWorld(Vector3(80, 62, -95), Vector3(15, 10, 1), 0.0f, Vector4(1, 0.5, 0.5, 1));
	AddCubeToWorld(Vector3(95, 62, 0), Vector3(1, 10, 15), 0.0f, Vector4(1, 0, 0.5, 1));
	AddCubeToWorld(Vector3(-30, 54, -50), Vector3(15, 2, 10), 0.0f, Vector4(1, 1, 0, 1), true, Vector3(20, 0, 0));
	AddCubeToWorld(Vector3(-30, 62, 60), Vector3(10, 2, 30), 0.0f, Vector4(1, 0, 0, 1), true, Vector3(-15, 0, 0));
	AddCubeToWorld(Vector3(-30, 62, 0), Vector3(10, 2, 30), 0.0f, Vector4(1, 0, 0, 1), true, Vector3(15, 0, 0));
	capsule = AddCapsuleToWorld(Vector3(-30, 55, 95), 10.0f, 3.0f, 0.0f, Vector3(0, 0, 90));
	capsule->setTrigger(true);
	checkpoints.emplace_back(AddCapsuleToWorld(Vector3(-70, 80, -84), 8.0f, 3.0f, 0.0, Vector3(0, 0, 90)));
	checkpoints.emplace_back(AddCapsuleToWorld(Vector3(80, 56, -90), 8.0f, 3.0f, 0.0, Vector3(0, 0, 90)));
	checkpoints.emplace_back(AddCapsuleToWorld(Vector3(30, 56, 0), 8.0f, 3.0f, 0.0, Vector3(0, 0, 90)));
	checkpoints.emplace_back(AddCapsuleToWorld(Vector3(-30, 56, -50), 8.0f, 3.0f, 0.0, Vector3(0, 0, 90)));
	for (int i = 0; i < checkpoints.size(); i++) {
		checkpoints[i]->setTrigger(true);
	}
	AddCapsuleToWorld(Vector3(0, 0, 0), 8.0f, 3.0f);
	AddCapsuleToWorld(Vector3(0, 0, 20), 8.0f, 3.0f);
}
void TutorialGame::InitWorld() {
	world->ClearAndErase();
	physics->Clear();
	InitMixedGridWorld(5, 5, 3.5f, 3.5f);
	testStateObject = AddStateObjectToWorld(Vector3(0, 10, 0));
	//BridgeConstraintTest();
	InitGameExamples();
	
	InitDefaultFloor();
	
}


/*

A single function to add a large immoveable cube to the bottom of our world

*/
GameObject* TutorialGame::AddFloorToWorld(const Vector3& position) {
	GameObject* floor = new GameObject();

	Vector3 floorSize	= Vector3(100, 2, 100);
	AABBVolume* volume	= new AABBVolume(floorSize);
	floor->SetBoundingVolume((CollisionVolume*)volume);
	floor->GetTransform()
		.SetScale(floorSize * 2)
		.SetPosition(position);

	floor->SetRenderObject(new RenderObject(&floor->GetTransform(), cubeMesh, basicTex, basicShader));
	floor->SetPhysicsObject(new PhysicsObject(&floor->GetTransform(), floor->GetBoundingVolume()));

	floor->GetPhysicsObject()->SetInverseMass(0);
	floor->GetPhysicsObject()->InitCubeInertia();
	floor->GetPhysicsObject()->setFriction(0.01f);
	world->AddGameObject(floor);

	return floor;
}

/*

Builds a game object that uses a sphere mesh for its graphics, and a bounding sphere for its
rigid body representation. This and the cube function will let you build a lot of 'simple' 
physics worlds. You'll probably need another function for the creation of OBB cubes too.

*/
GameObject* TutorialGame::AddSphereToWorld(const Vector3& position, float radius, bool gameBall, float inverseMass) {
	GameObject* sphere = new GameObject();

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume((CollisionVolume*)volume);

	sphere->GetTransform()
		.SetScale(sphereSize)
		.SetPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, basicTex, basicShader));
	sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));

	sphere->GetPhysicsObject()->SetInverseMass(inverseMass);
	sphere->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(sphere);
	if (gameBall) {
		Ball = sphere;
	}
	return sphere;
}

GameObject* TutorialGame::AddCapsuleToWorld(const Vector3& position, float halfHeight, float radius, float inverseMass, Vector3 angle) {
	GameObject* capsule = new GameObject();

	CapsuleVolume* volume = new CapsuleVolume(halfHeight, radius);
	capsule->SetBoundingVolume((CollisionVolume*)volume);

	capsule->GetTransform()
		.SetScale(Vector3(radius* 2, halfHeight, radius * 2))
		.SetPosition(position);
	if (angle.x > 0) {
		capsule->GetTransform().SetOrientation(Quaternion::AxisAngleToQuaterion(angle.Normalised(), angle.x));
	}
	else if (angle.x < 0) {
		capsule->GetTransform().SetOrientation(Quaternion::AxisAngleToQuaterion(angle.Normalised(), -angle.x));
	}
	if (angle.y > 0) {
		capsule->GetTransform().SetOrientation(Quaternion::AxisAngleToQuaterion(angle.Normalised(), angle.y));
	}
	else if (angle.y < 0) {
		capsule->GetTransform().SetOrientation(Quaternion::AxisAngleToQuaterion(angle.Normalised(), -angle.y));
	}
	if (angle.z > 0) {
		capsule->GetTransform().SetOrientation(Quaternion::AxisAngleToQuaterion(angle.Normalised(), angle.z));
	}
	else if (angle.z < 0) {
		capsule->GetTransform().SetOrientation(Quaternion::AxisAngleToQuaterion(angle.Normalised(), -angle.z));
	}
	capsule->SetRenderObject(new RenderObject(&capsule->GetTransform(), capsuleMesh, basicTex, basicShader));
	capsule->SetPhysicsObject(new PhysicsObject(&capsule->GetTransform(), capsule->GetBoundingVolume()));

	capsule->GetPhysicsObject()->SetInverseMass(inverseMass);
	capsule->GetPhysicsObject()->InitCubeInertia();
	
	world->AddGameObject(capsule);

	return capsule;

}

GameObject* TutorialGame::AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass, Vector4 colour, bool OBB, Vector3 angle) {
	GameObject* cube = new GameObject();
	if (OBB) {
		cube->SetBoundingVolume((CollisionVolume*)new OBBVolume(dimensions));
	}
	else {
		cube->SetBoundingVolume((CollisionVolume*)new AABBVolume(dimensions));
	}

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);
	if (angle.x > 0) {
		cube->GetTransform().SetOrientation(Quaternion::AxisAngleToQuaterion(angle.Normalised(), angle.x));
	}
	else if(angle.x <0){
		cube->GetTransform().SetOrientation(Quaternion::AxisAngleToQuaterion(angle.Normalised(), -angle.x));
	}
	if (angle.y > 0) {
		cube->GetTransform().SetOrientation(Quaternion::AxisAngleToQuaterion(angle.Normalised(), angle.y));
	}
	else if (angle.y < 0) {
		cube->GetTransform().SetOrientation(Quaternion::AxisAngleToQuaterion(angle.Normalised(), -angle.y));
	}
	if (angle.z > 0) {
		cube->GetTransform().SetOrientation(Quaternion::AxisAngleToQuaterion(angle.Normalised(), angle.z));
	}
	else if (angle.z < 0) {
		cube->GetTransform().SetOrientation(Quaternion::AxisAngleToQuaterion(angle.Normalised(), -angle.z));
	}
	if (colour == Vector4(1, 0, 0, 1)) {
		rotatingBridges.emplace_back(cube);
		rotatingBridgesAngles.emplace_back(angle);
	}
	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));
	cube->GetRenderObject()->SetColour(colour);
	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();
	if (colour == Vector4(0, 1, 0, 1)) {
		cube->GetPhysicsObject()->setElasticity(0.9f);
	}
	world->AddGameObject(cube);

	return cube;
}

void TutorialGame::InitSphereGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, float radius) {
	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
			AddSphereToWorld(position, radius, 1.0f);
		}
	}
	AddFloorToWorld(Vector3(0, -2, 0));
}

void TutorialGame::InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing) {
	float sphereRadius = 1.0f;
	Vector3 cubeDims = Vector3(1, 1, 1);

	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);

			if (rand() % 2) {
				AddCubeToWorld(position, cubeDims);
			}
			else {
				AddSphereToWorld(position, sphereRadius);
			}
		}
	}
}

void TutorialGame::InitCubeGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims) {
	for (int x = 1; x < numCols+1; ++x) {
		for (int z = 1; z < numRows+1; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
			AddCubeToWorld(position, cubeDims, 1.0f);
		}
	}
}

void TutorialGame::InitDefaultFloor() {
	AddFloorToWorld(Vector3(0, -2, 0));
}

void TutorialGame::InitGameExamples() {
	AddPlayerToWorld(Vector3(0, 5, 0));
	AddEnemyToWorld(Vector3(5, 5, 0));
	AddBonusToWorld(Vector3(10, 5, 0));
}

GameObject* TutorialGame::AddPlayerToWorld(const Vector3& position) {
	float meshSize = 3.0f;
	float inverseMass = 0.5f;

	GameObject* character = new GameObject();

	AABBVolume* volume = new AABBVolume(Vector3(0.3f, 0.85f, 0.3f) * meshSize);

	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	if (rand() % 2) {
		character->SetRenderObject(new RenderObject(&character->GetTransform(), charMeshA, nullptr, basicShader));
	}
	else {
		character->SetRenderObject(new RenderObject(&character->GetTransform(), charMeshB, nullptr, basicShader));
	}
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(character);

	//lockedObject = character;

	return character;
}

GameObject* TutorialGame::AddEnemyToWorld(const Vector3& position) {
	float meshSize		= 3.0f;
	float inverseMass	= 0.5f;

	GameObject* character = new GameObject();

	AABBVolume* volume = new AABBVolume(Vector3(0.3f, 0.9f, 0.3f) * meshSize);
	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	character->SetRenderObject(new RenderObject(&character->GetTransform(), enemyMesh, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(character);

	return character;
}

GameObject* TutorialGame::AddBonusToWorld(const Vector3& position) {
	GameObject* apple = new GameObject();

	SphereVolume* volume = new SphereVolume(1.2f);
	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform()
		.SetScale(Vector3(0.25, 0.25, 0.25))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), bonusMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(apple);

	return apple;
}
StateGameObject* TutorialGame::AddStateObjectToWorld(const Vector3& position) {
	StateGameObject* apple = new StateGameObject();

	SphereVolume* volume = new SphereVolume(1.2f);
	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform()
		.SetScale(Vector3(0.25, 0.25, 0.25))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), bonusMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(apple);

	return apple;
}

/*

Every frame, this code will let you perform a raycast, to see if there's an object
underneath the cursor, and if so 'select it' into a pointer, so that it can be 
manipulated later. Pressing Q will let you toggle between this behaviour and instead
letting you move the camera around. 

*/
bool TutorialGame::SelectObject() {
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::Q)) {
		inSelectionMode = !inSelectionMode;
		if (inSelectionMode) {
			Window::GetWindow()->ShowOSPointer(true);
			Window::GetWindow()->LockMouseToWindow(false);
		}
		else {
			Window::GetWindow()->ShowOSPointer(false);
			Window::GetWindow()->LockMouseToWindow(true);
		}
	}
	
	if (inSelectionMode) {
		renderer->DrawString("Press Q to change to camera mode!", Vector2(5, 85));
		if (selectionObject) {
			selectionObject->GetRenderObject()->SetColour(oColour);
			Vector3 objectPosition = selectionObject->GetTransform().GetPosition();
			renderer->DrawString("Position: " + to_string(objectPosition.x) + ", " + to_string(objectPosition.y) + ", " + to_string(objectPosition.z), Vector2(10, 10));
			Quaternion objectOrientation = selectionObject->GetTransform().GetOrientation();
			renderer->DrawString("Rotation: " + to_string(objectOrientation.x) + ", " + to_string(objectOrientation.y) + ", " + to_string(objectOrientation.z), Vector2(10, 15));
		}
		if (Window::GetMouse()->ButtonDown(NCL::MouseButtons::LEFT)) {
			if (selectionObject) {	//set colour to deselected;
				
				selectionObject = nullptr;
				lockedObject	= nullptr;
			}

			Ray ray = CollisionDetection::BuildRayFromMouse(*world->GetMainCamera());

			RayCollision closestCollision;
			if (world->Raycast(ray, closestCollision, true)) {
				selectionObject = (GameObject*)closestCollision.node;
				oColour = selectionObject->GetRenderObject()->GetColour();
				//selectionObject->GetRenderObject()->SetColour(Vector4(0, 1, 0, 1));
				return true;
			}
			else {
				return false;
			}
		}
	}
	else {
		renderer->DrawString("Press Q to change to select mode!", Vector2(5, 85));
	}

	if (lockedObject) {
		renderer->DrawString("Press L to unlock object!", Vector2(5, 80));
	}

	else if(selectionObject){
		renderer->DrawString("Press L to lock selected object object!", Vector2(5, 80));
	}

	if (Window::GetKeyboard()->KeyPressed(NCL::KeyboardKeys::L)) {
		if (selectionObject) {
			if (lockedObject == selectionObject) {
				lockedObject = nullptr;
			}
			else {
				lockedObject = selectionObject;
			}
		}

	}

	return false;
}

/*
If an object has been clicked, it can be pushed with the right mouse button, by an amount
determined by the scroll wheel. In the first tutorial this won't do anything, as we haven't
added linear motion into our physics system. After the second tutorial, objects will move in a straight
line - after the third, they'll be able to twist under torque aswell.
*/
void TutorialGame::AddRopeToBall() {
	if (activeRope) {
		world->RemoveConstraint(rope);
		rope = new PositionConstraint(tether, Ball, (tether->GetTransform().GetPosition() - Ball->GetTransform().GetPosition()).Length());
		world->AddConstraint(rope);
		Debug::DrawLine(tether->GetTransform().GetPosition(), Ball->GetTransform().GetPosition());
	}
	if (!activeRope) {
		world->RemoveConstraint(rope);
		rope = nullptr;
	}
}
void TutorialGame::RotateBridgeObjects() {
	if ((Ball->GetTransform().GetPosition().z - selectionObject->GetTransform().GetPosition().z)< 50) {
		Ball->GetTransform().SetPosition(Vector3(Ball->GetTransform().GetPosition().x, Ball->GetTransform().GetPosition().y + 20, Ball->GetTransform().GetPosition().z));
	}
	
	for (int i = 0; i < 4; i++) {

		if(rotatingBridgesAngles[i].x == 15){
			rotatingBridges[i]->GetTransform().SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(1, 0, 0), -15));
			rotatingBridgesAngles[i] = Vector3(-15, 0, 0);
		}
		else if(rotatingBridgesAngles[i].x == -15) {
			rotatingBridges[i]->GetTransform().SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(1, 0, 0), 15));
			rotatingBridgesAngles[i] = Vector3(15, 0, 0);
		}
	}
}
void TutorialGame::MoveSelectedObject() {
	//renderer->DrawString("Click Force: " + std::to_string(forceMagnitude), Vector2(10, 20));
	forceMagnitude += Window::GetMouse()->GetWheelMovement() * 100.0f;
	if (!selectionObject) {
		return;
	}
	if (Window::GetMouse()->ButtonPressed(NCL::MouseButtons::LEFT)) {
		Ray ray = CollisionDetection::BuildRayFromMouse(*world->GetMainCamera());
		RayCollision closestCollision;
		if (world->Raycast(ray, closestCollision, true)) {
			if (closestCollision.node == selectionObject) {
				if (selectionObject->GetRenderObject()->GetColour() == Vector4(1, 0, 0, 1)) {
					RotateBridgeObjects();
				}
				selectionObject->GetPhysicsObject()->AddForceAtPosition(ray.GetDirection() * forceMagnitude, closestCollision.collidedAt);
				if (selectionObject->GetRenderObject()->GetColour() == Vector4(0, 1, 0, 1)&& (Ball->GetTransform().GetPosition().y - (selectionObject->GetTransform().GetPosition().y + selectionObject->GetTransform().GetScale().y)) < 20 && (Ball->GetTransform().GetPosition().z - (selectionObject->GetTransform().GetPosition().z + selectionObject->GetTransform().GetScale().z)) < 10 && (Ball->GetTransform().GetPosition().x - (selectionObject->GetTransform().GetPosition().x + selectionObject->GetTransform().GetScale().x)) < 10)  {
					Ball->GetPhysicsObject()->AddForce(Vector3(0, 500, -10));
				}
				if (selectionObject->GetRenderObject()->GetColour() == Vector4(0, 0, 1, 1)) {
					selectionObject->GetPhysicsObject()->SetAngularVelocity(Vector3(0, -5, 0));
				}
				if (selectionObject->GetRenderObject()->GetColour() == Vector4(0.5, 1, 0, 1) ) {
					activeRope = !activeRope;
				}
				if (selectionObject->GetRenderObject()->GetColour() == Vector4(1, 0, 0.5, 1) &&  (Ball->GetTransform().GetPosition().z - (selectionObject->GetTransform().GetPosition().z + selectionObject->GetTransform().GetScale().z)) < 10 ) {
					Ball->GetPhysicsObject()->AddForce(Vector3(-500, 0, 0));
				}
				if (selectionObject->GetRenderObject()->GetColour() == Vector4(1, 0.5, 0.5, 1) && (Ball->GetTransform().GetPosition().x - (selectionObject->GetTransform().GetPosition().x + selectionObject->GetTransform().GetScale().x)) < 10) {
					Ball->GetPhysicsObject()->AddForce(Vector3(0, 0, 500));
				}
				if (selectionObject->GetRenderObject()->GetColour() == Vector4(1, 1, 0, 1) && (Ball->GetTransform().GetPosition().y - (selectionObject->GetTransform().GetPosition().y + selectionObject->GetTransform().GetScale().y)) < 20 && (Ball->GetTransform().GetPosition().z - (selectionObject->GetTransform().GetPosition().z + selectionObject->GetTransform().GetScale().z)) < 10 && (Ball->GetTransform().GetPosition().x - (selectionObject->GetTransform().GetPosition().x + selectionObject->GetTransform().GetScale().x)) < 10) {
					Ball->GetPhysicsObject()->AddForce(Vector3(0, 400, 250));
				}
				
			}
		}
	}
}