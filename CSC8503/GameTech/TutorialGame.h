#pragma once
#include "GameTechRenderer.h"
#include "../CSC8503Common/PhysicsSystem.h"
#include "../CSC8503Common/StateGameObject.h"

#include "../CSC8503Common/BehaviourAction.h"
#include "../CSC8503Common/BehaviourSequence.h"
#include "../CSC8503Common/BehaviourSelector.h"
namespace NCL {
	namespace CSC8503 {
		class TutorialGame		{
		public:
			TutorialGame();
			~TutorialGame();

			virtual void UpdateGame(float dt);
			GameTechRenderer* getRenderer() {
				return renderer;
			}
			Vector3 TestPathFinding();
			GameObject* FindClosestPowerUp();
			std::vector<Vector3> PathFinding(GameObject* target);
			void TestBehaviourTree();
			void DisplayPathfinding();
			void MoveEnemy();
			void Paused(float dt);
			void DrawMaze();
			void setGame(int g) { gameType = g; initialise = true; 	InitialiseAssets();
			}
			int getGame() { return gameType; }
			void CollectPowerUp();
			bool getCompleted() { return gameCompleted; }
			void End(float dt);
			bool getEndScreenShown() { return endScreenShown; }
			void reachCheckpoint();
			void setInitialise(bool i) { initialise = i; }
			int GetScore() { return score; }
		protected:
			std::chrono::steady_clock::time_point startTime;
			std::chrono::steady_clock::time_point endTime;
			std::vector<GameObject*> checkpoints;
			Vector3 lastCheckpoint = Vector3(-70, 75, 80);
			string gameTime;
			int score = 0;
			bool gameCompleted = false;
			bool endScreenShown = false;
			bool initialise = false;
			int gameType;
			void InitialiseAssets();

			void InitCamera();
			void UpdateKeys();

			void InitWorld();

			void InitGameExamples();
			
			void InitMap();
			void InitSphereGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, float radius);
			void InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing);
			void InitCubeGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims);
			void InitDefaultFloor();
			void BridgeConstraintTest();
	
			bool SelectObject();
			void MoveSelectedObject();
			void DebugObjectMovement();
			void LockedObjectMovement();

			GameObject* AddFloorToWorld(const Vector3& position);
			GameObject* AddSphereToWorld(const Vector3& position, float radius, bool gameBall = false, float inverseMass = 10.0f);
			GameObject* AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f, Vector4 colour = Vector4(1, 1, 1, 1),bool OBB = false, Vector3 angle = Vector3(0,0,0));
			GameObject* AddCapsuleToWorld(const Vector3& position, float halfHeight, float radius, float inverseMass = 10.0f, Vector3 angle = Vector3(0,0,0));

			GameObject* AddPlayerToWorld(const Vector3& position);
			GameObject* AddEnemyToWorld(const Vector3& position);
			GameObject* AddBonusToWorld(const Vector3& position);
			StateGameObject* AddStateObjectToWorld(const Vector3& position);

			GameTechRenderer*	renderer;
			PhysicsSystem*		physics;
			GameWorld*			world;

			bool useGravity;
			bool inSelectionMode;

			float		forceMagnitude;

			GameObject* selectionObject = nullptr;
			StateGameObject* testStateObject = nullptr;

			OGLMesh*	capsuleMesh = nullptr;
			OGLMesh*	cubeMesh	= nullptr;
			OGLMesh*	sphereMesh	= nullptr;
			OGLTexture* basicTex	= nullptr;
			OGLShader*	basicShader = nullptr;

			//Coursework Meshes
			OGLMesh*	charMeshA	= nullptr;
			OGLMesh*	charMeshB	= nullptr;
			OGLMesh*	enemyMesh	= nullptr;
			OGLMesh*	bonusMesh	= nullptr;

			//Coursework Additional functionality	
			GameObject* lockedObject	= nullptr;
			Vector3 lockedOffset		= Vector3(0, 14, 20);
			Vector4 oColour;
			void  RotateBridgeObjects();
			bool rotating = false;
			void LockCameraToObject(GameObject* o) {
				lockedObject = o;
			}
			std::vector<GameObject*> rotatingBridges;
			std::vector<Vector3> rotatingBridgesAngles;
			GameObject* Ball = nullptr;
			void AddRopeToBall();
			GameObject* tether = nullptr;
			bool activeRope = false;
			PositionConstraint* rope;
			GameObject* capsule = nullptr;
			GameObject* enemyBall = nullptr;
			GameObject* playerBall = nullptr;
			Vector3 aimPosition = Vector3(0,0,0);
			std::vector<GameObject*> powerups;
			GameObject* target;
			BehaviourState state = Ongoing;
			BehaviourSelector* selection = new BehaviourSelector("Selection");
		};
	}
}

