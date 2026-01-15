// This file contains the continuation of DX12Renderer.cpp
// Merge this content into DX12Renderer.cpp after the "// Continued in next part..." comment

bool DX12Renderer::CreateRootSignature()
{
    // Root parameter for constant buffer
    CD3DX12_ROOT_PARAMETER1 rootParameters[1];
    rootParameters[0].InitAsConstantBufferView(0, 0, D3D12_ROOT_DESCRIPTOR_FLAG_NONE, D3D12_SHADER_VISIBILITY_ALL);

    CD3DX12_VERSIONED_ROOT_SIGNATURE_DESC rootSignatureDesc;
    rootSignatureDesc.Init_1_1(_countof(rootParameters), rootParameters, 0, nullptr, D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

    ComPtr<ID3DBlob> signature;
    ComPtr<ID3DBlob> error;
    ThrowIfFailed(D3DX12SerializeVersionedRootSignature(&rootSignatureDesc, D3D_ROOT_SIGNATURE_VERSION_1, &signature, &error));
    ThrowIfFailed(device->CreateRootSignature(0, signature->GetBufferPointer(), signature->GetBufferSize(), IID_PPV_ARGS(&rootSignature)));

    return true;
}

bool DX12Renderer::LoadShaders()
{
    UINT compileFlags = D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION;

    // Get shader path
    wchar_t shaderPath[MAX_PATH];
    GetModuleFileNameW(nullptr, shaderPath, MAX_PATH);
    std::wstring exePath(shaderPath);
    size_t lastSlash = exePath.find_last_of(L"\\/");
    std::wstring shaderDir = exePath.substr(0, lastSlash) + L"\\..\\..\\shaders\\";

    std::wstring vsPath = shaderDir + L"PhysicsVS.hlsl";
    std::wstring psPath = shaderDir + L"PhysicsPS.hlsl";

    ComPtr<ID3DBlob> error;

    // Compile vertex shader
    HRESULT hr = D3DCompileFromFile(
        vsPath.c_str(),
        nullptr,
        D3D_COMPILE_STANDARD_FILE_INCLUDE,
        "main",
        "vs_5_0",
        compileFlags,
        0,
        &vertexShader,
        &error
    );

    if (FAILED(hr))
    {
        if (error)
        {
            std::cerr << "Vertex shader compilation error: " << (char*)error->GetBufferPointer() << std::endl;
        }
        throw std::runtime_error("Failed to compile vertex shader");
    }

    // Compile pixel shader
    hr = D3DCompileFromFile(
        psPath.c_str(),
        nullptr,
        D3D_COMPILE_STANDARD_FILE_INCLUDE,
        "main",
        "ps_5_0",
        compileFlags,
        0,
        &pixelShader,
        &error
    );

    if (FAILED(hr))
    {
        if (error)
        {
            std::cerr << "Pixel shader compilation error: " << (char*)error->GetBufferPointer() << std::endl;
        }
        throw std::runtime_error("Failed to compile pixel shader");
    }

    return true;
}

bool DX12Renderer::CreatePipelineState()
{
    // Input layout
    D3D12_INPUT_ELEMENT_DESC inputElementDescs[] =
    {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        { "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        { "COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 }
    };

    // Pipeline state object description
    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
    psoDesc.InputLayout = { inputElementDescs, _countof(inputElementDescs) };
    psoDesc.pRootSignature = rootSignature.Get();
    psoDesc.VS = CD3DX12_SHADER_BYTECODE(vertexShader.Get());
    psoDesc.PS = CD3DX12_SHADER_BYTECODE(pixelShader.Get());
    psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
    psoDesc.RasterizerState.CullMode = D3D12_CULL_MODE_BACK;
    psoDesc.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
    psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
    psoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
    psoDesc.SampleMask = UINT_MAX;
    psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    psoDesc.NumRenderTargets = 1;
    psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
    psoDesc.DSVFormat = DXGI_FORMAT_D32_FLOAT;
    psoDesc.SampleDesc.Count = 1;

    ThrowIfFailed(device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&pipelineState)));

    return true;
}

bool DX12Renderer::CreateCommandAllocatorsAndLists()
{
    for (UINT i = 0; i < 2; i++)
    {
        ThrowIfFailed(device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&commandAllocators[i])));
    }

    ThrowIfFailed(device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, commandAllocators[0].Get(), pipelineState.Get(), IID_PPV_ARGS(&commandList)));
    ThrowIfFailed(commandList->Close());

    return true;
}

void DX12Renderer::BeginFrame()
{
    // Reset command allocator and list
    ThrowIfFailed(commandAllocators[frameIndex]->Reset());
    ThrowIfFailed(commandList->Reset(commandAllocators[frameIndex].Get(), pipelineState.Get()));

    // Set necessary state
    commandList->SetGraphicsRootSignature(rootSignature.Get());

    ID3D12DescriptorHeap* ppHeaps[] = { cbvHeap.Get() };
    commandList->SetDescriptorHeaps(_countof(ppHeaps), ppHeaps);
    commandList->SetGraphicsRootConstantBufferView(0, constantBuffer->GetGPUVirtualAddress());

    commandList->RSSetViewports(1, &viewport);
    commandList->RSSetScissorRects(1, &scissorRect);

    // Transition render target to render target state
    commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(renderTargets[frameIndex].Get(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET));

    // Get render target and depth stencil handles
    CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(rtvHeap->GetCPUDescriptorHandleForHeapStart(), frameIndex, rtvDescriptorSize);
    CD3DX12_CPU_DESCRIPTOR_HANDLE dsvHandle(dsvHeap->GetCPUDescriptorHandleForHeapStart());

    commandList->OMSetRenderTargets(1, &rtvHandle, FALSE, &dsvHandle);

    // Clear render target and depth stencil
    const float clearColor[] = { 0.1f, 0.1f, 0.15f, 1.0f };
    commandList->ClearRenderTargetView(rtvHandle, clearColor, 0, nullptr);
    commandList->ClearDepthStencilView(dsvHandle, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);

    commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
}

void DX12Renderer::EndFrame()
{
    // Transition render target to present state
    commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(renderTargets[frameIndex].Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT));

    ThrowIfFailed(commandList->Close());

    // Execute command list
    ID3D12CommandList* ppCommandLists[] = { commandList.Get() };
    commandQueue->ExecuteCommandLists(_countof(ppCommandLists), ppCommandLists);

    // Present
    ThrowIfFailed(swapChain->Present(1, 0));

    MoveToNextFrame();
}

void DX12Renderer::WaitForGPU()
{
    if (commandQueue && fence)
    {
        const UINT64 fenceValue = fenceValues[frameIndex];
        ThrowIfFailed(commandQueue->Signal(fence.Get(), fenceValue));

        if (fence->GetCompletedValue() < fenceValue)
        {
            ThrowIfFailed(fence->SetEventOnCompletion(fenceValue, fenceEvent));
            WaitForSingleObject(fenceEvent, INFINITE);
        }

        fenceValues[frameIndex]++;
    }
}

void DX12Renderer::MoveToNextFrame()
{
    const UINT64 currentFenceValue = fenceValues[frameIndex];
    ThrowIfFailed(commandQueue->Signal(fence.Get(), currentFenceValue));

    frameIndex = swapChain->GetCurrentBackBufferIndex();

    if (fence->GetCompletedValue() < fenceValues[frameIndex])
    {
        ThrowIfFailed(fence->SetEventOnCompletion(fenceValues[frameIndex], fenceEvent));
        WaitForSingleObject(fenceEvent, INFINITE);
    }

    fenceValues[frameIndex] = currentFenceValue + 1;
}

void DX12Renderer::UpdateConstantBuffer(const XMMATRIX& world)
{
    ConstantBuffer cb;

    XMMATRIX view = camera.GetViewMatrix();
    XMMATRIX proj = camera.GetProjectionMatrix();

    cb.worldViewProj = XMMatrixTranspose(world * view * proj);
    cb.world = XMMatrixTranspose(world);
    cb.lightDir = XMFLOAT4(0.3f, -0.7f, 0.3f, 0.0f);
    cb.cameraPos = XMFLOAT4(camera.GetPosition().x, camera.GetPosition().y, camera.GetPosition().z, 1.0f);

    memcpy(constantBufferData, &cb, sizeof(cb));
}

void DX12Renderer::SetCamera(const Camera& cam)
{
    camera = cam;
}

void DX12Renderer::Resize(UINT width, UINT height)
{
    this->width = width;
    this->height = height;
    // TODO: Implement resize logic
}

// Continued in Part 3...
