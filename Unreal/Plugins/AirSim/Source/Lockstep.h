// VICTECH

#pragma once
#include "CoreMinimal.h"
#include <mutex>
#include <condition_variable>

class FLockstep
{
public:
	FLockstep();
	~FLockstep();

	void SetEnabled();

	bool IsEnabled() const
	{
		return isEnabled_;
	}

	void Callback_OnEndFrame();
	void Lockstep();

private:
	bool isEnabled_{ false };
	std::mutex mtx_;
	std::condition_variable cv_;
	bool isGameThreadRunning_{ true };
};
extern FLockstep GLockstep;

