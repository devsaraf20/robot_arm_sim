// ArmOscDriverComponent.cpp — see the header and unreal/UNREAL.md.

#include "ArmOscDriverComponent.h"

#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "OSCClient.h"
#include "OSCManager.h"
#include "OSCServer.h"

UArmOscDriverComponent::UArmOscDriverComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UArmOscDriverComponent::BeginPlay()
{
	Super::BeginPlay();

	// OSC "client" = sender. It fires UDP packets at the bridge.
	Client = UOSCManager::CreateOSCClient(BridgeHost, BridgePort, TEXT("RobotArmClient"), this);

	// OSC "server" = receiver, for the arm state the simulator sends back.
	if (bReceiveArmState)
	{
		Server = UOSCManager::CreateOSCServer(TEXT("127.0.0.1"), ListenPort, false, true, TEXT("RobotArmServer"), this);
		if (Server)
		{
			Server->OnOscMessageReceived.AddDynamic(this, &UArmOscDriverComponent::HandleOscMessage);
		}
	}
}

void UArmOscDriverComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (Server)
	{
		Server->Stop();
		Server = nullptr;
	}
	Client = nullptr;
	Super::EndPlay(EndPlayReason);
}

FVector UArmOscDriverComponent::GetTargetRelativeCm() const
{
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return FVector::ZeroVector;
	}

	FVector World = Owner->GetActorLocation();
	if (!SocketName.IsNone())
	{
		if (const USkeletalMeshComponent* Mesh = Owner->FindComponentByClass<USkeletalMeshComponent>())
		{
			World = Mesh->GetSocketLocation(SocketName);
		}
	}

	// Express the point in the arm base's own frame: X forward, Y right, Z up (cm).
	// InverseTransformPositionNoScale ignores the base actor's scale.
	const FVector Local = ArmBase ? ArmBase->GetActorTransform().InverseTransformPositionNoScale(World) : World;
	return Local * WorkspaceScale;
}

void UArmOscDriverComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!bSendTarget || !Client)
	{
		return;
	}

	// Send at SendRateHz, not every frame: no point flooding the network at 144 fps.
	SendAccumulator += DeltaTime;
	if (SendAccumulator < 1.0f / SendRateHz)
	{
		return;
	}
	SendAccumulator = 0.0f;

	const FVector T = GetTargetRelativeCm();
	Send(TEXT("/arm/target"), { static_cast<float>(T.X), static_cast<float>(T.Y), static_cast<float>(T.Z) });
}

void UArmOscDriverComponent::Send(const FString& Address, const TArray<float>& Values)
{
	if (!Client)
	{
		return;
	}
	FOSCMessage Message;
	UOSCManager::SetOSCMessageAddress(Message, UOSCManager::ConvertStringToOSCAddress(Address));
	for (const float V : Values)
	{
		UOSCManager::AddFloat(Message, V);
	}
	Client->SendOSCMessage(Message);
}

void UArmOscDriverComponent::SendEstop() { Send(TEXT("/arm/estop"), {}); }
void UArmOscDriverComponent::SendHome() { Send(TEXT("/arm/home"), {}); }
void UArmOscDriverComponent::SetGripper(float Open01) { Send(TEXT("/arm/gripper"), { FMath::Clamp(Open01, 0.0f, 1.0f) }); }

void UArmOscDriverComponent::SendJoints(float Base, float Shoulder, float Elbow, float Gripper)
{
	Send(TEXT("/arm/joints"), { Base, Shoulder, Elbow, Gripper });
}

void UArmOscDriverComponent::HandleOscMessage(const FOSCMessage& Message, const FString& IPAddress, int32 Port)
{
	const FString Address = UOSCManager::GetOSCAddressFullPath(UOSCManager::GetOSCMessageAddress(Message));

	if (Address == TEXT("/arm/state"))
	{
		TArray<float> V;
		UOSCManager::GetAllFloats(Message, V);
		if (V.Num() >= 4)
		{
			OnArmState.Broadcast(V[0], V[1], V[2], V[3]);
		}
	}
	else if (Address == TEXT("/arm/tip"))
	{
		TArray<float> V;
		UOSCManager::GetAllFloats(Message, V);
		if (V.Num() >= 3)
		{
			OnArmTip.Broadcast(FVector(V[0], V[1], V[2]));
		}
	}
	else if (Address == TEXT("/arm/status"))
	{
		TArray<FString> S;
		UOSCManager::GetAllStrings(Message, S);
		if (S.Num() > 0)
		{
			OnArmStatus.Broadcast(S[0]);
		}
	}
}
