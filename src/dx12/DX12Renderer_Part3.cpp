// This file contains the final part of DX12Renderer.cpp
// Merge this content after Part 2

void DX12Renderer::RenderPhysXScene(PxScene* scene)
{
    if (!scene) return;

    // Get all actors
    PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
    if (nbActors == 0) return;

    std::vector<PxRigidActor*> actors(nbActors);
    scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC,
                     reinterpret_cast<PxActor**>(&actors[0]), nbActors);

    // Render each actor
    for (PxU32 i = 0; i < nbActors; i++)
    {
        PxRigidActor* actor = actors[i];

        // Determine color based on actor type
        XMFLOAT3 color(0.5f, 0.5f, 0.5f);
        if (actor->is<PxRigidDynamic>())
        {
            color = XMFLOAT3(0.3f, 0.7f, 0.3f); // Green for dynamic
        }
        else if (actor->is<PxRigidStatic>())
        {
            color = XMFLOAT3(0.6f, 0.6f, 0.6f); // Gray for static
        }

        RenderActor(actor, color);
    }
}

void DX12Renderer::RenderActor(PxRigidActor* actor, const XMFLOAT3& color)
{
    if (!actor) return;

    PxU32 nShapes = actor->getNbShapes();
    std::vector<PxShape*> shapes(nShapes);
    actor->getShapes(&shapes[0], nShapes);

    for (PxU32 i = 0; i < nShapes; i++)
    {
        PxShape* shape = shapes[i];

        // Get transform
        PxTransform pxTransform = PxShapeExt::getGlobalPose(*shape, *actor);
        PxMat44 pxMat(pxTransform);

        // Convert PhysX matrix to DirectXMath
        XMMATRIX world = XMMatrixSet(
            pxMat[0][0], pxMat[0][1], pxMat[0][2], pxMat[0][3],
            pxMat[1][0], pxMat[1][1], pxMat[1][2], pxMat[1][3],
            pxMat[2][0], pxMat[2][1], pxMat[2][2], pxMat[2][3],
            pxMat[3][0], pxMat[3][1], pxMat[3][2], pxMat[3][3]
        );

        // Update constant buffer with world transform
        UpdateConstantBuffer(world);

        // Get geometry and render
        PxGeometryType::Enum geomType = shape->getGeometryType();

        switch (geomType)
        {
        case PxGeometryType::eBOX:
        {
            PxBoxGeometry boxGeom;
            shape->getBoxGeometry(boxGeom);
            RenderMesh mesh = CreateBoxMesh(boxGeom, color);

            commandList->IASetVertexBuffers(0, 1, &mesh.vertexBufferView);
            commandList->IASetIndexBuffer(&mesh.indexBufferView);
            commandList->DrawIndexedInstanced(mesh.indexCount, 1, 0, 0, 0);
            break;
        }
        case PxGeometryType::eSPHERE:
        {
            PxSphereGeometry sphereGeom;
            shape->getSphereGeometry(sphereGeom);
            RenderMesh mesh = CreateSphereMesh(sphereGeom, color);

            commandList->IASetVertexBuffers(0, 1, &mesh.vertexBufferView);
            commandList->IASetIndexBuffer(&mesh.indexBufferView);
            commandList->DrawIndexedInstanced(mesh.indexCount, 1, 0, 0, 0);
            break;
        }
        case PxGeometryType::ePLANE:
        {
            if (!groundMesh)
            {
                groundMesh = std::make_unique<RenderMesh>(CreatePlaneMesh(color));
            }

            commandList->IASetVertexBuffers(0, 1, &groundMesh->vertexBufferView);
            commandList->IASetIndexBuffer(&groundMesh->indexBufferView);
            commandList->DrawIndexedInstanced(groundMesh->indexCount, 1, 0, 0, 0);
            break;
        }
        default:
            // Unsupported geometry type
            break;
        }
    }
}

RenderMesh DX12Renderer::CreateBoxMesh(const PxBoxGeometry& geometry, const XMFLOAT3& color)
{
    RenderMesh mesh;
    mesh.color = color;

    PxVec3 halfExtents = geometry.halfExtents;

    // Box vertices (8 corners)
    Vertex vertices[] =
    {
        // Front face
        { XMFLOAT3(-halfExtents.x, -halfExtents.y,  halfExtents.z), XMFLOAT3(0, 0, 1), color },
        { XMFLOAT3( halfExtents.x, -halfExtents.y,  halfExtents.z), XMFLOAT3(0, 0, 1), color },
        { XMFLOAT3( halfExtents.x,  halfExtents.y,  halfExtents.z), XMFLOAT3(0, 0, 1), color },
        { XMFLOAT3(-halfExtents.x,  halfExtents.y,  halfExtents.z), XMFLOAT3(0, 0, 1), color },
        // Back face
        { XMFLOAT3( halfExtents.x, -halfExtents.y, -halfExtents.z), XMFLOAT3(0, 0, -1), color },
        { XMFLOAT3(-halfExtents.x, -halfExtents.y, -halfExtents.z), XMFLOAT3(0, 0, -1), color },
        { XMFLOAT3(-halfExtents.x,  halfExtents.y, -halfExtents.z), XMFLOAT3(0, 0, -1), color },
        { XMFLOAT3( halfExtents.x,  halfExtents.y, -halfExtents.z), XMFLOAT3(0, 0, -1), color },
        // Top face
        { XMFLOAT3(-halfExtents.x,  halfExtents.y,  halfExtents.z), XMFLOAT3(0, 1, 0), color },
        { XMFLOAT3( halfExtents.x,  halfExtents.y,  halfExtents.z), XMFLOAT3(0, 1, 0), color },
        { XMFLOAT3( halfExtents.x,  halfExtents.y, -halfExtents.z), XMFLOAT3(0, 1, 0), color },
        { XMFLOAT3(-halfExtents.x,  halfExtents.y, -halfExtents.z), XMFLOAT3(0, 1, 0), color },
        // Bottom face
        { XMFLOAT3(-halfExtents.x, -halfExtents.y, -halfExtents.z), XMFLOAT3(0, -1, 0), color },
        { XMFLOAT3( halfExtents.x, -halfExtents.y, -halfExtents.z), XMFLOAT3(0, -1, 0), color },
        { XMFLOAT3( halfExtents.x, -halfExtents.y,  halfExtents.z), XMFLOAT3(0, -1, 0), color },
        { XMFLOAT3(-halfExtents.x, -halfExtents.y,  halfExtents.z), XMFLOAT3(0, -1, 0), color },
        // Right face
        { XMFLOAT3( halfExtents.x, -halfExtents.y,  halfExtents.z), XMFLOAT3(1, 0, 0), color },
        { XMFLOAT3( halfExtents.x, -halfExtents.y, -halfExtents.z), XMFLOAT3(1, 0, 0), color },
        { XMFLOAT3( halfExtents.x,  halfExtents.y, -halfExtents.z), XMFLOAT3(1, 0, 0), color },
        { XMFLOAT3( halfExtents.x,  halfExtents.y,  halfExtents.z), XMFLOAT3(1, 0, 0), color },
        // Left face
        { XMFLOAT3(-halfExtents.x, -halfExtents.y, -halfExtents.z), XMFLOAT3(-1, 0, 0), color },
        { XMFLOAT3(-halfExtents.x, -halfExtents.y,  halfExtents.z), XMFLOAT3(-1, 0, 0), color },
        { XMFLOAT3(-halfExtents.x,  halfExtents.y,  halfExtents.z), XMFLOAT3(-1, 0, 0), color },
        { XMFLOAT3(-halfExtents.x,  halfExtents.y, -halfExtents.z), XMFLOAT3(-1, 0, 0), color },
    };

    UINT indices[] =
    {
        0, 1, 2,  0, 2, 3,    // Front
        4, 5, 6,  4, 6, 7,    // Back
        8, 9, 10, 8, 10, 11,  // Top
        12, 13, 14, 12, 14, 15, // Bottom
        16, 17, 18, 16, 18, 19, // Right
        20, 21, 22, 20, 22, 23  // Left
    };

    mesh.indexCount = _countof(indices);

    // Create vertex buffer
    const UINT vertexBufferSize = sizeof(vertices);
    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(vertexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.vertexBuffer)
    ));

    UINT8* pVertexDataBegin;
    CD3DX12_RANGE readRange(0, 0);
    ThrowIfFailed(mesh.vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pVertexDataBegin)));
    memcpy(pVertexDataBegin, vertices, vertexBufferSize);
    mesh.vertexBuffer->Unmap(0, nullptr);

    mesh.vertexBufferView.BufferLocation = mesh.vertexBuffer->GetGPUVirtualAddress();
    mesh.vertexBufferView.StrideInBytes = sizeof(Vertex);
    mesh.vertexBufferView.SizeInBytes = vertexBufferSize;

    // Create index buffer
    const UINT indexBufferSize = sizeof(indices);
    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(indexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.indexBuffer)
    ));

    UINT8* pIndexDataBegin;
    ThrowIfFailed(mesh.indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pIndexDataBegin)));
    memcpy(pIndexDataBegin, indices, indexBufferSize);
    mesh.indexBuffer->Unmap(0, nullptr);

    mesh.indexBufferView.BufferLocation = mesh.indexBuffer->GetGPUVirtualAddress();
    mesh.indexBufferView.Format = DXGI_FORMAT_R32_UINT;
    mesh.indexBufferView.SizeInBytes = indexBufferSize;

    return mesh;
}

RenderMesh DX12Renderer::CreateSphereMesh(const PxSphereGeometry& geometry, const XMFLOAT3& color)
{
    RenderMesh mesh;
    mesh.color = color;

    const int segments = 16;
    const int rings = 16;
    const float radius = geometry.radius;

    std::vector<Vertex> vertices;
    std::vector<UINT> indices;

    // Generate sphere vertices
    for (int ring = 0; ring <= rings; ring++)
    {
        float phi = XM_PI * ring / rings;
        float y = radius * cosf(phi);
        float ringRadius = radius * sinf(phi);

        for (int segment = 0; segment <= segments; segment++)
        {
            float theta = 2.0f * XM_PI * segment / segments;
            float x = ringRadius * cosf(theta);
            float z = ringRadius * sinf(theta);

            XMFLOAT3 position(x, y, z);
            XMFLOAT3 normal = position;
            XMStoreFloat3(&normal, XMVector3Normalize(XMLoadFloat3(&normal)));

            vertices.push_back({ position, normal, color });
        }
    }

    // Generate sphere indices
    for (int ring = 0; ring < rings; ring++)
    {
        for (int segment = 0; segment < segments; segment++)
        {
            int current = ring * (segments + 1) + segment;
            int next = current + segments + 1;

            indices.push_back(current);
            indices.push_back(next);
            indices.push_back(current + 1);

            indices.push_back(current + 1);
            indices.push_back(next);
            indices.push_back(next + 1);
        }
    }

    mesh.indexCount = static_cast<UINT>(indices.size());

    // Create vertex buffer
    const UINT vertexBufferSize = static_cast<UINT>(vertices.size() * sizeof(Vertex));
    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(vertexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.vertexBuffer)
    ));

    UINT8* pVertexDataBegin;
    CD3DX12_RANGE readRange(0, 0);
    ThrowIfFailed(mesh.vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pVertexDataBegin)));
    memcpy(pVertexDataBegin, vertices.data(), vertexBufferSize);
    mesh.vertexBuffer->Unmap(0, nullptr);

    mesh.vertexBufferView.BufferLocation = mesh.vertexBuffer->GetGPUVirtualAddress();
    mesh.vertexBufferView.StrideInBytes = sizeof(Vertex);
    mesh.vertexBufferView.SizeInBytes = vertexBufferSize;

    // Create index buffer
    const UINT indexBufferSize = static_cast<UINT>(indices.size() * sizeof(UINT));
    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(indexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.indexBuffer)
    ));

    UINT8* pIndexDataBegin;
    ThrowIfFailed(mesh.indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pIndexDataBegin)));
    memcpy(pIndexDataBegin, indices.data(), indexBufferSize);
    mesh.indexBuffer->Unmap(0, nullptr);

    mesh.indexBufferView.BufferLocation = mesh.indexBuffer->GetGPUVirtualAddress();
    mesh.indexBufferView.Format = DXGI_FORMAT_R32_UINT;
    mesh.indexBufferView.SizeInBytes = indexBufferSize;

    return mesh;
}

RenderMesh DX12Renderer::CreatePlaneMesh(const XMFLOAT3& color)
{
    RenderMesh mesh;
    mesh.color = color;

    const float size = 50.0f;

    Vertex vertices[] =
    {
        { XMFLOAT3(-size, 0, -size), XMFLOAT3(0, 1, 0), color },
        { XMFLOAT3( size, 0, -size), XMFLOAT3(0, 1, 0), color },
        { XMFLOAT3( size, 0,  size), XMFLOAT3(0, 1, 0), color },
        { XMFLOAT3(-size, 0,  size), XMFLOAT3(0, 1, 0), color },
    };

    UINT indices[] = { 0, 1, 2, 0, 2, 3 };

    mesh.indexCount = _countof(indices);

    // Create vertex buffer
    const UINT vertexBufferSize = sizeof(vertices);
    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(vertexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.vertexBuffer)
    ));

    UINT8* pVertexDataBegin;
    CD3DX12_RANGE readRange(0, 0);
    ThrowIfFailed(mesh.vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pVertexDataBegin)));
    memcpy(pVertexDataBegin, vertices, vertexBufferSize);
    mesh.vertexBuffer->Unmap(0, nullptr);

    mesh.vertexBufferView.BufferLocation = mesh.vertexBuffer->GetGPUVirtualAddress();
    mesh.vertexBufferView.StrideInBytes = sizeof(Vertex);
    mesh.vertexBufferView.SizeInBytes = vertexBufferSize;

    // Create index buffer
    const UINT indexBufferSize = sizeof(indices);
    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(indexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.indexBuffer)
    ));

    UINT8* pIndexDataBegin;
    ThrowIfFailed(mesh.indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pIndexDataBegin)));
    memcpy(pIndexDataBegin, indices, indexBufferSize);
    mesh.indexBuffer->Unmap(0, nullptr);

    mesh.indexBufferView.BufferLocation = mesh.indexBuffer->GetGPUVirtualAddress();
    mesh.indexBufferView.Format = DXGI_FORMAT_R32_UINT;
    mesh.indexBufferView.SizeInBytes = indexBufferSize;

    return mesh;
}
