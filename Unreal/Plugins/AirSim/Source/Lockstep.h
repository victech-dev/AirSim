// VICTECH

#pragma once
#include "CoreMinimal.h"
#include <mutex>
#include <condition_variable>
#include "SimMode/SimModeWorldBase.h"

class FLockstep
{
public:
	FLockstep();
	~FLockstep();

	bool IsEnabled() const
	{
		return FApp::UseFixedTimeStep();
	}

	void Initialize(ASimModeBase* simmode);

	void Callback_OnEndFrame();
	void Lockstep();

private:
	ASimModeBase* simmode_{ nullptr };
	int physicsUpdatePerFrame_{ 0 };
	std::mutex mtx_;
	std::condition_variable cv_;
	bool isGameThreadRunning_{ true };
};
extern FLockstep GLockstep;

