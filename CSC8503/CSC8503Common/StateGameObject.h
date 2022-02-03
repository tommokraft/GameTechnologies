#pragma once
#include "GameObject.h"
namespace NCL {
	namespace CSC8503 {
		class StateMachine;
		class StateGameObject : public GameObject {
		public:
			StateGameObject();
			~StateGameObject();
			virtual void Update(float dt);
		protected:
			void MoveLeft(float dt);
			void MoveRight(float dt);
			void MoveToPlayer();
			void MoveToPowerUp();
			StateMachine * stateMachine;
			float counter;
		};
	}
}

