cmake_minimum_required(VERSION 3.16)
project(PhysXSample)

set(CMAKE_CXX_STANDARD 14)
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

# ビルドタイプに応じたライブラリディレクトリ
if(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(PHYSX_BUILD_TYPE "debug")
elseif(CMAKE_BUILD_TYPE MATCHES "Release")
    set(PHYSX_BUILD_TYPE "release")
else()
    set(PHYSX_BUILD_TYPE "debug")
endif()

link_directories(${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE})

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
add_executable(PhysXSample src/main.cpp)
add_executable(PhysXSampleOmniPVD src/main_omnipvd.cpp)
add_executable(PhysXSampleDX12 WIN32
    src/main_dx12.cpp
    src/dx12/DX12Renderer.cpp
    src/dx12/DX12Renderer.h
    src/dx12/d3dx12.h
)

# PhysXライブラリをリンク
target_link_libraries(PhysXSample
    PhysX_64
    PhysXCommon_64
    PhysXFoundation_64
    PhysXExtensions_static_64
    PhysXPvdSDK_static_64
)

target_link_libraries(PhysXSampleOmniPVD
    PhysX_64
    PhysXCommon_64
    PhysXFoundation_64
    PhysXExtensions_static_64
    PhysXPvdSDK_static_64
    PVDRuntime_64
)

# DirectX 12サンプル用のライブラリリンク
target_link_libraries(PhysXSampleDX12
    PhysX_64
    PhysXCommon_64
    PhysXFoundation_64
    PhysXExtensions_static_64
    ImGui
    d3d12.lib
    dxgi.lib
    d3dcompiler.lib
)

# ランタイムDLLをコピー
add_custom_command(TARGET PhysXSample POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE}/PhysX_64.dll"
        $<TARGET_FILE_DIR:PhysXSample>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE}/PhysXCommon_64.dll"
        $<TARGET_FILE_DIR:PhysXSample>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE}/PhysXFoundation_64.dll"
        $<TARGET_FILE_DIR:PhysXSample>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE}/PhysXCooking_64.dll"
        $<TARGET_FILE_DIR:PhysXSample>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE}/PVDRuntime_64.dll"
        $<TARGET_FILE_DIR:PhysXSample>
)

add_custom_command(TARGET PhysXSampleOmniPVD POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE}/PhysX_64.dll"
        $<TARGET_FILE_DIR:PhysXSampleOmniPVD>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE}/PhysXCommon_64.dll"
        $<TARGET_FILE_DIR:PhysXSampleOmniPVD>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE}/PhysXFoundation_64.dll"
        $<TARGET_FILE_DIR:PhysXSampleOmniPVD>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE}/PhysXCooking_64.dll"
        $<TARGET_FILE_DIR:PhysXSampleOmniPVD>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE}/PVDRuntime_64.dll"
        $<TARGET_FILE_DIR:PhysXSampleOmniPVD>
)

add_custom_command(TARGET PhysXSampleDX12 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE}/PhysX_64.dll"
        $<TARGET_FILE_DIR:PhysXSampleDX12>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE}/PhysXCommon_64.dll"
        $<TARGET_FILE_DIR:PhysXSampleDX12>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE}/PhysXFoundation_64.dll"
        $<TARGET_FILE_DIR:PhysXSampleDX12>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE}/PhysXCooking_64.dll"
        $<TARGET_FILE_DIR:PhysXSampleDX12>
    # Copy GPU PhysX DLLs if they exist (optional)
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE}/PhysXGpu_64.dll"
        $<TARGET_FILE_DIR:PhysXSampleDX12> || ${CMAKE_COMMAND} -E echo "PhysXGpu_64.dll not found - GPU PhysX will be disabled"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PHYSX_LIB_DIR}/${PHYSX_BUILD_TYPE}/PhysXDevice64.dll"
        $<TARGET_FILE_DIR:PhysXSampleDX12> || ${CMAKE_COMMAND} -E echo "PhysXDevice64.dll not found - GPU PhysX will be disabled"
    # シェーダーファイルをコピー
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:PhysXSampleDX12>/../shaders
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/PhysicsVS.hlsl"
        $<TARGET_FILE_DIR:PhysXSampleDX12>/../shaders/
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/PhysicsPS.hlsl"
        $<TARGET_FILE_DIR:PhysXSampleDX12>/../shaders/
)

# Visual Studioのスタートアッププロジェクトに設定
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT PhysXSampleDX12)
