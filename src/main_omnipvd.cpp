#include <PxPhysicsAPI.h>
#include <iostream>
#include <vector>

#if PX_SUPPORT_OMNI_PVD
#include "omnipvd/PxOmniPvd.h"
#endif

using namespace physx;

// グローバル変数
PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;
PxFoundation*			gFoundation = nullptr;
PxPhysics*				gPhysics = nullptr;
PxDefaultCpuDispatcher*	gDispatcher = nullptr;
PxScene*				gScene = nullptr;
PxMaterial*				gMaterial = nullptr;

#if PX_SUPPORT_OMNI_PVD
PxOmniPvd*				gOmniPvd = nullptr;
#endif

// 初期化関数（OmniPVD対応）
bool initPhysics(const char* pvdFileName = "physx_simulation.ovd")
{
	// Foundation作成
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	if (!gFoundation) {
		std::cerr << "PxCreateFoundation failed!" << std::endl;
		return false;
	}

#if PX_SUPPORT_OMNI_PVD
	// OmniPVD初期化
	gOmniPvd = PxCreateOmniPvd(*gFoundation);
	if (gOmniPvd)
	{
		OmniPvdWriter* omniWriter = gOmniPvd->getWriter();
		if (omniWriter)
		{
			OmniPvdFileWriteStream* fStream = gOmniPvd->getFileWriteStream();
			if (fStream)
			{
				fStream->setFileName(pvdFileName);
				omniWriter->setWriteStream(*fStream);
				std::cout << "OmniPVD will write to: " << pvdFileName << std::endl;
			}
		}
	}
#endif

	// Physics作成（OmniPVDを渡す）
#if PX_SUPPORT_OMNI_PVD
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, nullptr, gOmniPvd);
#else
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, nullptr);
#endif

	if (!gPhysics) {
		std::cerr << "PxCreatePhysics failed!" << std::endl;
		return false;
	}

#if PX_SUPPORT_OMNI_PVD
	// OmniPVDサンプリング開始
	if (gPhysics->getOmniPvd())
	{
		if (gPhysics->getOmniPvd()->startSampling())
		{
			std::cout << "OmniPVD sampling started successfully!" << std::endl;
		}
		else
		{
			std::cerr << "Warning: Could not start OmniPVD sampling" << std::endl;
		}
	}
#endif

	// シーン作成
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	// マテリアル作成
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	std::cout << "PhysX initialized successfully!" << std::endl;
	return true;
}

// 地面を作成
PxRigidStatic* createGround()
{
	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);
	return groundPlane;
}

// 動的な箱を作成
PxRigidDynamic* createBox(const PxVec3& position, const PxVec3& halfExtents, float density = 1.0f)
{
	PxTransform transform(position);
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, transform,
		PxBoxGeometry(halfExtents), *gMaterial, density);
	gScene->addActor(*dynamic);
	return dynamic;
}

// 球を作成
PxRigidDynamic* createSphere(const PxVec3& position, float radius, float density = 1.0f)
{
	PxTransform transform(position);
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, transform,
		PxSphereGeometry(radius), *gMaterial, density);
	gScene->addActor(*dynamic);
	return dynamic;
}

// シミュレーション実行
void stepPhysics(float deltaTime)
{
	gScene->simulate(deltaTime);
	gScene->fetchResults(true);
}

// クリーンアップ
void cleanupPhysics()
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
#if PX_SUPPORT_OMNI_PVD
	PX_RELEASE(gOmniPvd);
#endif
	PX_RELEASE(gFoundation);

	std::cout << "PhysX cleaned up." << std::endl;
}

int main(int argc, char** argv)
{
	std::cout << "=== PhysX Sample with OmniPVD ===" << std::endl;

	// 出力ファイル名
	const char* pvdFileName = "physx_simulation.ovd";
	if (argc > 1) {
		pvdFileName = argv[1];
	}

	// PhysX初期化
	if (!initPhysics(pvdFileName)) {
		return -1;
	}

	// 地面を作成
	createGround();
	std::cout << "Ground plane created." << std::endl;

	// さまざまなオブジェクトを作成
	std::vector<PxRigidDynamic*> objects;

	// 箱を作成
	objects.push_back(createBox(PxVec3(0, 10, 0), PxVec3(0.5f, 0.5f, 0.5f)));
	objects.push_back(createBox(PxVec3(2, 15, 0), PxVec3(0.5f, 0.5f, 0.5f)));
	objects.push_back(createBox(PxVec3(-2, 20, 0), PxVec3(0.5f, 0.5f, 0.5f)));

	// 球を作成
	objects.push_back(createSphere(PxVec3(1, 25, 1), 0.5f));
	objects.push_back(createSphere(PxVec3(-1, 30, -1), 0.5f));

	// 大きな箱を作成
	objects.push_back(createBox(PxVec3(0, 35, 0), PxVec3(1.0f, 1.0f, 1.0f)));

	std::cout << "Created " << objects.size() << " dynamic objects." << std::endl;

	// シミュレーション実行
	const float timeStep = 1.0f / 60.0f;
	const int numSteps = 600; // 10秒間のシミュレーション

	std::cout << "\nStarting simulation (10 seconds)..." << std::endl;
	std::cout << "Recording to OmniPVD file: " << pvdFileName << std::endl;

	for (int i = 0; i < numSteps; i++)
	{
		stepPhysics(timeStep);

		// 100フレームごとに進捗を出力
		if (i % 100 == 0)
		{
			std::cout << "Frame " << i << " / " << numSteps
			          << " (Time: " << i * timeStep << "s)" << std::endl;
		}
	}

	std::cout << "\nSimulation completed!" << std::endl;
	std::cout << "OmniPVD data saved to: " << pvdFileName << std::endl;

	// クリーンアップ
	cleanupPhysics();

	std::cout << "\nTo visualize the simulation:" << std::endl;
	std::cout << "1. Install NVIDIA Omniverse" << std::endl;
	std::cout << "2. Open the OVD file (" << pvdFileName << ") in Omniverse" << std::endl;

	return 0;
}
