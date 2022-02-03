#include "../../Common/Window.h"

#include "../CSC8503Common/StateMachine.h"
#include "../CSC8503Common/StateTransition.h"
#include "../CSC8503Common/State.h"

#include "../CSC8503Common/NavigationGrid.h"

#include "TutorialGame.h"
#include "../CSC8503Common/BehaviourNode.h"
#include "..//CSC8503Common/BehaviourAction.h"
#include "../CSC8503Common/BehaviourSequence.h"
#include "../CSC8503Common/BehaviourSelector.h"
#include "..//CSC8503Common/PushdownState.h"
#include "../CSC8503Common/PushdownMachine.h"

using namespace NCL;
using namespace CSC8503;
/*

The main function should look pretty familar to you!
We make a window, and then go into a while loop that repeatedly
runs our 'game' until we press escape. Instead of making a 'renderer'
and updating it, we instead make a whole game, and repeatedly update that,
instead. 

This time, we've added some extra functionality to the window class - we can
hide or show the 

*/


void TestStateMachine() {
	StateMachine* testMachine = new StateMachine();
	int data = 0;
	State* A = new State([&](float dt)->void
		{
			std::cout << "Im in state A" << std::endl;
			data++;
		}
	);
	State* B = new State([&](float dt)->void
		{
			std::cout << "Im in state B" << std::endl;
			data--;
		}
	);
	StateTransition* stateAB = new StateTransition(A, B, [&](void)->bool {
		return data > 10;
		}
	);
	StateTransition* stateBA = new StateTransition(B, A, [&](void)->bool {
		return data < 0;
		}
	);
	testMachine->AddState(A);
	testMachine->AddState(B);
	testMachine->AddTransition(stateAB);
	testMachine->AddTransition(stateBA);
	for (int i = 0; i < 100; ++i) {
		testMachine->Update(1.0f);
	}
}

class EndScreen : public PushdownState {
public:
	EndScreen(TutorialGame* g) { game = g; }
	PushdownResult OnUpdate(float dt, PushdownState** newState) override {
		game->End(dt);
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F1)) {
			return PushdownResult::Pop;
		}
		return PushdownResult::NoChange;
	}
	void OnAwake() override {

	}
protected:
	TutorialGame* game;
};
class PauseScreen : public PushdownState {
public:
	PauseScreen(TutorialGame* g) { game = g; }
	PushdownResult OnUpdate(float dt, PushdownState** newState) override {
		game->Paused(dt);
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::U)) {
			return PushdownResult::Pop;
		}
		return PushdownResult::NoChange;
	}
	void OnAwake() override {
		std::cout << "Press U to unpause game" << std::endl;
	}
protected:
	TutorialGame* game;
};
class GameScreen : public PushdownState {
public:
	GameScreen(TutorialGame* game) { g = game; }
	PushdownResult OnUpdate(float dt, PushdownState** newState) override {
		if (g->getCompleted()) {
			*newState = new EndScreen(g);
			return PushdownResult::Push;
		}
		if (g->getEndScreenShown()) {
			return PushdownResult::Pop;
		}
		
		g->getRenderer()->DrawString("Press P To Pause or F1 to Return To Main Menu  Or R To Reset The Ball", Vector2(2, 5),Vector4(0.75,0.75,0.75,1),15.0f);
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::P)) {
			
			*newState = new PauseScreen(g);
			return PushdownResult::Push;
		}
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::F1)) {
			g->setInitialise(false);
			return PushdownResult::Pop;
		}
		g->UpdateGame(dt);
		return PushdownResult::NoChange;
	};
	void OnAwake() override {
	}
protected:
	TutorialGame* g; 
};
class IntroScreen : public PushdownState {
	PushdownResult OnUpdate(float dt, PushdownState** newState) override {
		
		g->UpdateGame(dt);
		g->getRenderer()->DrawString("Press 1 To Play Physics Game", Vector2(20, 50));
		g->getRenderer()->DrawString("Press 2 To Play AI Game,", Vector2(20, 60));
		g->getRenderer()->DrawString("Escape To Quit", Vector2(20, 70));
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::NUM1)) {
			g->setGame(1);
			*newState = new GameScreen(g);
			return PushdownResult::Push;
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::NUM2)) {
			g->setGame(2);
			*newState = new GameScreen(g);
			return PushdownResult::Push;
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::ESCAPE)) {
			return PushdownResult::Pop;
		}
		return PushdownResult::NoChange;
	}
	void OnAwake() override {
		g = new TutorialGame();
		
	}
protected:
	TutorialGame* g = nullptr;
};
void GamePushdownAutomata(Window* w) {
	PushdownMachine machine(new IntroScreen());
	float dt = w->GetTimer()->GetTimeDeltaSeconds();

	while (w->UpdateWindow() && !(Window::GetKeyboard()->KeyPressed(KeyboardKeys::ESCAPE))) {
		float dt = w->GetTimer()->GetTimeDeltaSeconds();
		if (dt > 0.1f) {
			std::cout << "Skipping large time delta" << std::endl;
			continue; //must have hit a breakpoint or something to have a 1 second frame time!
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::PRIOR)) {
			w->ShowConsole(true);
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::NEXT)) {
			w->ShowConsole(false);
		}

		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::T)) {
			w->SetWindowPosition(0, 0);
		}

		w->SetTitle("Gametech frame time:" + std::to_string(1000.0f * dt));
		if (!machine.Update(dt)) {
			return;
		}
	}
	
}
int main() {
	Window* w = Window::CreateGameWindow("CSC8503 Game technology!", 1280, 720);

	if (!w->HasInitialised()) {
		return -1;
	}	
	srand(time(0));
	//w->ShowOSPointer(false);
	//w->LockMouseToWindow(true);

	
	w->GetTimer()->GetTimeDeltaSeconds(); //Clear the timer so we don't get a larget first dt!
	


	//TestBehaviourTree();

	//TestStateMachine();
	GamePushdownAutomata(w);
	/*float dt = w->GetTimer()->GetTimeDeltaSeconds();
	TutorialGame* game = new TutorialGame();
	while (w->UpdateWindow() && !(Window::GetKeyboard()->KeyPressed(KeyboardKeys::ESCAPE))) {
		float dt = w->GetTimer()->GetTimeDeltaSeconds();
		if (dt > 0.1f) {
			std::cout << "Skipping large time delta" << std::endl;
			continue; //must have hit a breakpoint or something to have a 1 second frame time!
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::PRIOR)) {
			w->ShowConsole(true);
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::NEXT)) {
			w->ShowConsole(false);
		}

		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::T)) {
			w->SetWindowPosition(0, 0);
		}
		game->UpdateGame(dt);
		w->SetTitle("Gametech frame time:" + std::to_string(1000.0f * dt));
	}*/
		//DisplayPathfinding();
		//TestPushdownAutomata(w);

	Window::DestroyGameWindow();
}