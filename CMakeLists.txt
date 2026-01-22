cmake_minimum_required(VERSION 3.16)
project(PhysXWorkbench)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# PhysXと同じ静的ランタイムライブラリを使用
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")

# PhysX SDKのパス
set(PHYSX_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../PhysX-5.6.1/physx" CACHE PATH "PhysX root directory")

# インクルードディレクトリ
include_directories(
    ${PHYSX_ROOT_DIR}/include
    ${PHYSX_ROOT_DIR}/source/foundation/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui/backends
)

# ライブラリディレクトリ
set(PHYSX_LIB_DIR "${PHYSX_ROOT_DIR}/bin/win.x86_64.vc143.mt")

# Note: link_directories() は使用しない（マルチ構成ジェネレータでは正しく動作しない）
# 代わりに、target_link_libraries() でジェネレータ式を使用してフルパスを指定する

# ImGui ライブラリ
add_library(ImGui STATIC
    external/imgui/imgui.cpp
    external/imgui/imgui_demo.cpp
    external/imgui/imgui_draw.cpp
    external/imgui/imgui_tables.cpp
    external/imgui/imgui_widgets.cpp
    external/imgui/backends/imgui_impl_win32.cpp
    external/imgui/backends/imgui_impl_dx12.cpp
)

# 実行ファイル作成
add_executable(PhysXWorkbenchConsole src/main.cpp)
add_executable(PhysXWorkbench WIN32
    src/main_dx12.cpp
    src/dx12/DX12Renderer.cpp
    src/dx12/DX12Renderer.h
    src/dx12/d3dx12.h
    src/simulation/SimulationRecorder.cpp
    src/simulation/SimulationRecorder.h
    src/simulation/PerformanceProfiler.cpp
    src/simulation/PerformanceProfiler.h
    src/simulation/ExperimentController.cpp
    src/simulation/ExperimentController.h
    src/simulation/SceneLoader.cpp
    src/simulation/SceneLoader.h
    src/simulation/DeformableVolumeManager.cpp
    src/simulation/DeformableVolumeManager.h
    src/CommandLineArgs.cpp
    src/CommandLineArgs.h
    # Dynamic Bonding System
    src/simulation/bonding/PropertyMap.h
    src/simulation/bonding/BondingSiteDef.h
    src/simulation/bonding/BondableEntityDef.h
    src/simulation/bonding/BondableEntity.h
    src/simulation/bonding/BondableEntity.cpp
    src/simulation/bonding/Bond.h
    src/simulation/bonding/IBondFormationRule.h
    src/simulation/bonding/BondFormationRules.h
    src/simulation/bonding/BondFormationRules.cpp
    src/simulation/bonding/IBondType.h
    src/simulation/bonding/BondTypes.h
    src/simulation/bonding/BondTypes.cpp
    src/simulation/bonding/DynamicBondManager.h
    src/simulation/bonding/DynamicBondManager.cpp
    src/simulation/bonding/BondingIntegration.h
    src/simulation/bonding/DemoBondingScenes.h
    # Batch Simulation System
    src/simulation/batch/IMetric.h
    src/simulation/batch/CommonMetrics.h
    src/simulation/batch/CommonMetrics.cpp
    src/simulation/batch/ITerminationCondition.h
    src/simulation/batch/CommonTerminationConditions.h
    src/simulation/batch/CommonTerminationConditions.cpp
    src/simulation/batch/BatchSimulationRunner.h
    src/simulation/batch/BatchSimulationRunner.cpp
    src/simulation/batch/MSERSteadyStateCondition.h
    src/simulation/batch/MSERSteadyStateCondition.cpp
)

# PhysXライブラリをリンク（ジェネレータ式でDebug/Release構成に対応）
target_link_libraries(PhysXWorkbenchConsole
    $<$<CONFIG:Debug>:${PHYSX_LIB_DIR}/debug/PhysX_64.lib>
    $<$<CONFIG:Release>:${PHYSX_LIB_DIR}/release/PhysX_64.lib>
    $<$<CONFIG:Debug>:${PHYSX_LIB_DIR}/debug/PhysXCommon_64.lib>
    $<$<CONFIG:Release>:${PHYSX_LIB_DIR}/release/PhysXCommon_64.lib>
    $<$<CONFIG:Debug>:${PHYSX_LIB_DIR}/debug/PhysXFoundation_64.lib>
    $<$<CONFIG:Release>:${PHYSX_LIB_DIR}/release/PhysXFoundation_64.lib>
    $<$<CONFIG:Debug>:${PHYSX_LIB_DIR}/debug/PhysXExtensions_static_64.lib>
    $<$<CONFIG:Release>:${PHYSX_LIB_DIR}/release/PhysXExtensions_static_64.lib>
    $<$<CONFIG:Debug>:${PHYSX_LIB_DIR}/debug/PhysXPvdSDK_static_64.lib>
    $<$<CONFIG:Release>:${PHYSX_LIB_DIR}/release/PhysXPvdSDK_static_64.lib>
)

# DirectX 12 メインアプリケーション用のライブラリリンク
target_link_libraries(PhysXWorkbench
    $<$<CONFIG:Debug>:${PHYSX_LIB_DIR}/debug/PhysX_64.lib>
    $<$<CONFIG:Release>:${PHYSX_LIB_DIR}/release/PhysX_64.lib>
    $<$<CONFIG:Debug>:${PHYSX_LIB_DIR}/debug/PhysXCommon_64.lib>
    $<$<CONFIG:Release>:${PHYSX_LIB_DIR}/release/PhysXCommon_64.lib>
    $<$<CONFIG:Debug>:${PHYSX_LIB_DIR}/debug/PhysXFoundation_64.lib>
    $<$<CONFIG:Release>:${PHYSX_LIB_DIR}/release/PhysXFoundation_64.lib>
    $<$<CONFIG:Debug>:${PHYSX_LIB_DIR}/debug/PhysXExtensions_static_64.lib>
    $<$<CONFIG:Release>:${PHYSX_LIB_DIR}/release/PhysXExtensions_static_64.lib>
    $<$<CONFIG:Debug>:${PHYSX_LIB_DIR}/debug/PhysXPvdSDK_static_64.lib>
    $<$<CONFIG:Release>:${PHYSX_LIB_DIR}/release/PhysXPvdSDK_static_64.lib>
    $<$<CONFIG:Debug>:${PHYSX_LIB_DIR}/debug/PhysXCooking_64.lib>
    $<$<CONFIG:Release>:${PHYSX_LIB_DIR}/release/PhysXCooking_64.lib>
    ImGui
    d3d12.lib
    dxgi.lib
    d3dcompiler.lib
)

# ランタイムDLLをコピー
add_custom_command(TARGET PhysXWorkbenchConsole POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/$<CONFIG>/PhysX_64.dll"
        $<TARGET_FILE_DIR:PhysXWorkbenchConsole>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/$<CONFIG>/PhysXCommon_64.dll"
        $<TARGET_FILE_DIR:PhysXWorkbenchConsole>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/$<CONFIG>/PhysXFoundation_64.dll"
        $<TARGET_FILE_DIR:PhysXWorkbenchConsole>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/$<CONFIG>/PhysXCooking_64.dll"
        $<TARGET_FILE_DIR:PhysXWorkbenchConsole>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/$<CONFIG>/PVDRuntime_64.dll"
        $<TARGET_FILE_DIR:PhysXWorkbenchConsole>
)

add_custom_command(TARGET PhysXWorkbench POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/$<CONFIG>/PhysX_64.dll"
        $<TARGET_FILE_DIR:PhysXWorkbench>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/$<CONFIG>/PhysXCommon_64.dll"
        $<TARGET_FILE_DIR:PhysXWorkbench>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/$<CONFIG>/PhysXFoundation_64.dll"
        $<TARGET_FILE_DIR:PhysXWorkbench>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/$<CONFIG>/PhysXCooking_64.dll"
        $<TARGET_FILE_DIR:PhysXWorkbench>
    # Copy GPU PhysX DLL (PhysX 5.x uses PhysXGpu_64.dll)
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/$<CONFIG>/PhysXGpu_64.dll"
        $<TARGET_FILE_DIR:PhysXWorkbench>
    # シェーダーファイルをコピー
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:PhysXWorkbench>/../shaders
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/PhysicsVS.hlsl"
        $<TARGET_FILE_DIR:PhysXWorkbench>/../shaders/
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/PhysicsPS.hlsl"
        $<TARGET_FILE_DIR:PhysXWorkbench>/../shaders/
)

# Visual Studioのスタートアッププロジェクトに設定
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT PhysXWorkbench)
