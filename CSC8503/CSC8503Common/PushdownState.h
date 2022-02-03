#pragma once
#include "State.h"
#include "../GameTech/TutorialGame.h"
namespace NCL {
	namespace CSC8503 {
		class PushdownState {
		public:
			enum PushdownResult {
				Push, Pop, NoChange
			};
			PushdownState();
			virtual ~PushdownState();

			virtual PushdownResult OnUpdate(float dt,PushdownState** pushFunc) = 0;

			virtual void OnAwake() {} //By default do nothing
			virtual void OnSleep() {} //By default do nothing
			void SetGame(TutorialGame* g) {
				game = g;
			}
			TutorialGame* GetGame() {
				return game;
			}
		protected:
			TutorialGame* game;
		};
	}
}

