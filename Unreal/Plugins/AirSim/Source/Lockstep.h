// VICTECH

#pragma once
#include "CoreMinimal.h"
#include <mutex>
#include <condition_variable>
#include "SimMode/SimModeWorldBase.h"
#include "common/WorkerThread.hpp"

class FLockstep : public msr::airlib::SteppableClock
{
public:
	typedef msr::airlib::TTimePoint TTimePoint;
	typedef msr::airlib::TTimeDelta TTimeDelta;
	typedef msr::airlib::WaiterSyncSignal WaiterSyncSignal;

	FLockstep(TTimeDelta step, TTimePoint start = 0);
	virtual ~FLockstep();

	static void Initialize(ASimModeBase* simmode);

	// called from UE GameThread
	void Callback_OnEndFrame();
	// called from external API (like python)
	void Lockstep(bool paused = false);

	// Physics/ControlCommand update
	void WorldTick(msr::airlib::PhysicsWorld& world, float deltaTime);

	// Restore view mode if simGetImages requested
	void RestoreViewMode();

	// from SteppableClock
	virtual bool isLockstepMode() const override
	{
		return true;
	}
	virtual void registerWaiter(std::shared_ptr<WaiterSyncSignal> waiter_signal, TTimeDelta period) override;
	virtual void unregisterWaiter(std::shared_ptr<WaiterSyncSignal> waiter_signal) override;
	virtual void signalCanceledWaiter() override;
	virtual void sleep_for(TTimeDelta dt) override
	{
		throw std::runtime_error("Not implemented!");
	}

private:
	ASimModeBase* simmode_{ nullptr };
	std::mutex mtx_;
	std::condition_variable cv_;
	bool isGameThreadRunning_{ true };
	double frameDeltaTime_;
	bool isSimModeWorld_;

	// event members/functions
	enum class EventType
	{
		kPhysics,	// 3ms period
		kWaiter,	// 20ms period for control command
	};
	struct Event
	{
		EventType type;
		TTimePoint time;
		TTimeDelta period;
		std::shared_ptr<WaiterSyncSignal> waiter_signal;
	};
	std::vector<Event> events_;
	std::mutex eventMutex_;

	bool PopEvent(Event& front, TTimePoint until);
	void PushEvent(EventType type, TTimePoint time, TTimeDelta period, std::shared_ptr<WaiterSyncSignal> waiter_signal);
	void RegisterPhysicsEvent(TTimeDelta period);
};

extern std::shared_ptr<FLockstep> GLockstep;

