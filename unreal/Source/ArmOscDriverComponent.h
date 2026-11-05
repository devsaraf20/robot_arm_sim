// ArmOscDriverComponent.h — drive the Robot Arm Twin from Unreal Engine over OSC.
//
// Add this component to ANY actor (a sphere, a character, a motion-controller pawn).
// Every tick it sends the actor's location — or a skeletal-mesh socket such as
// "hand_r" — to the browser simulator as /arm/target, relative to an "ArmBase" actor.
// The simulator solves IK and, if connected, moves the real arm.
// It also listens for /arm/state so Blueprints can animate an in-engine twin.
//
// Requires the built-in "OSC" plugin (Edit → Plugins → OSC) and "OSC" in your
// module's Build.cs dependencies. Written against UE 5.3+.
// Replace YOURPROJECT_API with your project's API macro (e.g. MYGAME_API).

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "OSCMessage.h"
#include "ArmOscDriverComponent.generated.h"

class UOSCClient;
class UOSCServer;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_FourParams(FOnArmState, float, Base, float, Shoulder, float, Elbow, float, Gripper);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnArmTip, FVector, TipRelativeToBaseCm);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnArmStatus, const FString&, Status);

UCLASS(ClassGroup = (Robotics), meta = (BlueprintSpawnableComponent))
class YOURPROJECT_API UArmOscDriverComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UArmOscDriverComponent();

	// ── What to follow ───────────────────────────────────────────────────────
	/** The arm's base in the level. Targets are sent relative to it (cm, X forward, Z up). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot Arm")
	TObjectPtr<AActor> ArmBase;

	/** If set, follow this socket/bone on the owner's skeletal mesh (e.g. "hand_r"). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot Arm")
	FName SocketName = NAME_None;

	/** Multiplies the offset from ArmBase. The desktop arm reaches ~30 cm, so to map a
	 *  person-sized motion (~2 m) onto it use ~0.15. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot Arm", meta = (ClampMin = "0.001"))
	float WorkspaceScale = 1.0f;

	/** Send /arm/target every tick (turn off to only use SendJoints / SetGripper). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot Arm")
	bool bSendTarget = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot Arm", meta = (ClampMin = "1", ClampMax = "120"))
	float SendRateHz = 30.0f;

	// ── Network ───────────────────────────────────────────────────────────────
	/** Where the OSC bridge (npm run bridge) listens. */
	UPROPERTY(EditAnywhere, Category = "Robot Arm|Network")
	FString BridgeHost = TEXT("127.0.0.1");

	UPROPERTY(EditAnywhere, Category = "Robot Arm|Network")
	int32 BridgePort = 8000;

	/** Receive /arm/state, /arm/tip and /arm/status back from the simulator. */
	UPROPERTY(EditAnywhere, Category = "Robot Arm|Network")
	bool bReceiveArmState = true;

	UPROPERTY(EditAnywhere, Category = "Robot Arm|Network")
	int32 ListenPort = 8001;

	// ── Events (bind in Blueprint) ────────────────────────────────────────────
	/** Joint angles in simulator degrees. In Unreal: base → Yaw = -Base, shoulder → Pitch, elbow → Pitch. */
	UPROPERTY(BlueprintAssignable, Category = "Robot Arm")
	FOnArmState OnArmState;

	UPROPERTY(BlueprintAssignable, Category = "Robot Arm")
	FOnArmTip OnArmTip;

	UPROPERTY(BlueprintAssignable, Category = "Robot Arm")
	FOnArmStatus OnArmStatus;

	// ── Commands ───────────────────────────────────────────────────────────────
	/** Emergency stop. Unreal can trigger it; only a person in the browser can reset it. */
	UFUNCTION(BlueprintCallable, Category = "Robot Arm")
	void SendEstop();

	UFUNCTION(BlueprintCallable, Category = "Robot Arm")
	void SendHome();

	/** 0 = closed, 1 = fully open. */
	UFUNCTION(BlueprintCallable, Category = "Robot Arm")
	void SetGripper(float Open01);

	/** Direct joint control in simulator degrees (switches the sim to sliders mode). */
	UFUNCTION(BlueprintCallable, Category = "Robot Arm")
	void SendJoints(float Base, float Shoulder, float Elbow, float Gripper);

	/** The point currently being sent, relative to ArmBase, in cm (after WorkspaceScale). */
	UFUNCTION(BlueprintPure, Category = "Robot Arm")
	FVector GetTargetRelativeCm() const;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	void Send(const FString& Address, const TArray<float>& Values);

	UFUNCTION()
	void HandleOscMessage(const FOSCMessage& Message, const FString& IPAddress, int32 Port);

	UPROPERTY(Transient)
	TObjectPtr<UOSCClient> Client;

	UPROPERTY(Transient)
	TObjectPtr<UOSCServer> Server;

	float SendAccumulator = 0.0f;
};
