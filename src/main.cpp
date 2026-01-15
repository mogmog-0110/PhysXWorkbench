#include <PxPhysicsAPI.h>
#include <iostream>
#include <vector>

using namespace physx;

// グローバル変数
PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;
PxFoundation*			gFoundation = nullptr;
PxPhysics*				gPhysics = nullptr;
PxDefaultCpuDispatcher*	gDispatcher = nullptr;
PxScene*				gScene = nullptr;
PxMaterial*				gMaterial = nullptr;
PxPvd*					gPvd = nullptr;

// 初期化関数
bool initPhysics()
{
	// Foundation作成
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	if (!gFoundation) {
		std::cerr << "PxCreateFoundation failed!" << std::endl;
		return false;
	}

	// PVD (PhysX Visual Debugger) 初期化
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	// Physics作成
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	if (!gPhysics) {
		std::cerr << "PxCreatePhysics failed!" << std::endl;
		return false;
	}

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
	if (gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();
		gPvd = nullptr;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

	std::cout << "PhysX cleaned up." << std::endl;
}

int main()
{
	std::cout << "=== PhysX Sample Program ===" << std::endl;

	// PhysX初期化
	if (!initPhysics()) {
		return -1;
	}

	// 地面を作成
	createGround();
	std::cout << "Ground plane created." << std::endl;

	// 空中に箱を作成（落下させる）
	std::vector<PxRigidDynamic*> boxes;
	boxes.push_back(createBox(PxVec3(0, 10, 0), PxVec3(0.5f, 0.5f, 0.5f)));
	boxes.push_back(createBox(PxVec3(1, 15, 0), PxVec3(0.5f, 0.5f, 0.5f)));
	boxes.push_back(createBox(PxVec3(-1, 20, 0), PxVec3(0.5f, 0.5f, 0.5f)));
	std::cout << "Created " << boxes.size() << " boxes." << std::endl;

	// シミュレーション実行
	const float timeStep = 1.0f / 60.0f;
	const int numSteps = 300;

	std::cout << "\nStarting simulation..." << std::endl;
	for (int i = 0; i < numSteps; i++)
	{
		stepPhysics(timeStep);

		// 50フレームごとに位置を出力
		if (i % 50 == 0)
		{
			std::cout << "\n--- Frame " << i << " (Time: " << i * timeStep << "s) ---" << std::endl;
			for (size_t j = 0; j < boxes.size(); j++)
			{
				PxVec3 pos = boxes[j]->getGlobalPose().p;
				std::cout << "Box " << j << ": Y = " << pos.y << std::endl;
			}
		}
	}

	std::cout << "\nSimulation completed!" << std::endl;

	// クリーンアップ
	cleanupPhysics();

	return 0;
}
