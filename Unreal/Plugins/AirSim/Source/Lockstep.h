// VICTECH

#pragma once
#include "CoreMinimal.h"
#include <mutex>
#include <condition_variable>
#include "api/ApiProvider.hpp"

class FLockstep
{
public:
	FLockstep();
	~FLockstep();

	void SetEnabled(msr::airlib::ApiProvider* apiProvider);

	bool IsEnabled() const
	{
		return isEnabled_;
	}

	void Callback_OnEndFrame();
	void Lockstep();

private:
	bool isEnabled_{ false };
	msr::airlib::ApiProvider* apiProvider_{ nullptr };
	std::mutex mtx_;
	std::condition_variable cv_;
	bool isGameThreadRunning_{ true };
};
extern FLockstep GLockstep;

