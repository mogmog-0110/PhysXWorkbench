# PhysX + DirectX 12 リアルタイム可視化

PhysX 5.6.1とDirectX 12を統合した高性能リアルタイム物理シミュレーション可視化システムです。

## 特徴

### 高性能レンダリング
- **DirectX 12**: 最新のWindows低レベルグラフィックスAPI
- **GPU最適化**: 最小限のCPUオーバーヘッド
- **60 FPS**: リアルタイム物理演算と同期した滑らかな描画

### 高品質ビジュアル
- **Blinn-Phongライティング**: リアルな光沢とスペキュラーハイライト
- **ディレクショナルライト**: 方向性照明による立体感
- **アンビエントライト**: 環境光による自然な見た目

### インタラクティブ操作
- **WASDキー**: カメラ移動（前後左右）
- **Q/Eキー**: カメラ上下移動
- **右マウスボタン**: 視点回転（ドラッグ）
- **ESC**: 終了

## ビルド方法

### 前提条件
- PhysX 5.6.1 SDK (Debugビルド)
- DirectX 12対応GPU
- Windows 10/11
- Visual Studio 2022
- CMake 3.16以上

### ビルド手順

```bash
cd PhysXSample
cmake -G "Visual Studio 17 2022" -A x64 -B build
cd build
cmake --build . --config Debug --target PhysXSampleDX12
```

## 実行方法

```bash
cd PhysXSample/build/Debug
PhysXSampleDX12.exe
```

### 実行結果

3Dウィンドウが開き、以下のシミュレーションが表示されます：

- **地面**: 静的な無限平面（グレー）
- **箱3個**: 高さ10m, 15m, 20mから落下（緑色）
- **球2個**: 高さ25m, 30mから落下（緑色）
- **大きな箱1個**: 高さ35mから落下（緑色）

すべてのオブジェクトが重力により落下し、地面に衝突してリアルに跳ね返ります。

## 技術詳細

### アーキテクチャ

```
PhysXSampleDX12
├── PhysX Physics Engine
│   ├── PxFoundation
│   ├── PxPhysics
│   ├── PxScene
│   └── Rigid Bodies (Dynamic/Static)
│
└── DirectX 12 Renderer
    ├── Device & Command Queue
    ├── Swap Chain (Double Buffering)
    ├── Pipeline State Object
    ├── Root Signature
    ├── Constant Buffer (MVP Matrix, Lighting)
    ├── Vertex/Index Buffers
    └── Depth Stencil Buffer
```

### シェーダーパイプライン

1. **Vertex Shader (PhysicsVS.hlsl)**
   - モデル座標 → ワールド座標 → ビュー座標 → クリップ座標
   - 法線ベクトルのワールド変換
   - 頂点カラーの転送

2. **Pixel Shader (PhysicsPS.hlsl)**
   - Blinn-Phongライティング計算
   - Diffuse (拡散反射) + Specular (鏡面反射) + Ambient (環境光)
   - 最終カラー出力

### パフォーマンス

- **物理演算**: 60 FPS固定タイムステップ (1/60秒)
- **レンダリング**: VSync有効、60 FPS
- **GPU使用率**: 低～中程度（オブジェクト数に依存）

## プログラムのカスタマイズ

### オブジェクトの追加

`main_dx12.cpp`の`WinMain`関数内で追加：

```cpp
// カプセルを追加
objects.push_back(createCapsule(PxVec3(0, 40, 0), 0.5f, 1.0f));
```

### カメラの初期位置変更

`DX12Renderer.cpp`の`Initialize`関数内：

```cpp
camera.SetPosition(0.0f, 20.0f, -40.0f);  // より遠くから
camera.SetLookAt(0.0f, 10.0f, 0.0f);      // 高い位置を見る
```

### ライティングの調整

`shaders/PhysicsPS.hlsl`を編集：

```hlsl
// より明るいアンビエント
float3 ambient = float3(0.4f, 0.4f, 0.4f);

// より強いスペキュラー
float specular = pow(max(dot(normal, halfDir), 0.0f), 64.0f); // 64に変更
```

### 重力の変更

`main_dx12.cpp`の`initPhysics`関数内：

```cpp
sceneDesc.gravity = PxVec3(0.0f, -19.62f, 0.0f); // 2倍の重力
// または
sceneDesc.gravity = PxVec3(0.0f, -1.62f, 0.0f);  // 月の重力
```

## トラブルシューティング

### ウィンドウが開かない

**原因**: DirectX 12非対応GPU、またはドライバーが古い

**解決方法**:
- グラフィックスドライバーを最新版に更新
- `dxdiag`コマンドでDirectX 12サポートを確認

### シェーダーコンパイルエラー

```
Failed to compile vertex shader
```

**原因**: シェーダーファイルが見つからない、またはパスが間違っている

**解決方法**:
- `build/shaders/`ディレクトリにHLSLファイルが存在するか確認
- CMakeを再実行してファイルをコピー

### PhysX DLLが見つからない

```
PhysX_64.dll が見つかりません
```

**解決方法**:
```bash
# PhysX DLLを手動でコピー
copy ..\PhysX-5.6.1\physx\bin\win.x86_64.vc143.mt\debug\*.dll build\Debug\
```

### 低フレームレート

**原因**: オブジェクト数が多すぎる、またはGPUが非力

**解決方法**:
- オブジェクト数を減らす
- ウィンドウサイズを小さくする（`WINDOW_WIDTH/HEIGHT`を変更）
- シェーダーの品質を下げる（スペキュラーを無効化など）

## ファイル構成

```
PhysXSample/
├── src/
│   ├── main_dx12.cpp              # メインエントリーポイント
│   └── dx12/
│       ├── DX12Renderer.h         # レンダラーヘッダー
│       ├── DX12Renderer.cpp       # レンダラー実装
│       └── d3dx12.h               # DirectX 12ヘルパー
├── shaders/
│   ├── PhysicsVS.hlsl             # 頂点シェーダー
│   └── PhysicsPS.hlsl             # ピクセルシェーダー
├── build/
│   ├── Debug/
│   │   ├── PhysXSampleDX12.exe    # 実行ファイル
│   │   └── *.dll                   # PhysX DLL
│   └── shaders/                    # コンパイル済みシェーダー
├── CMakeLists.txt                  # ビルド設定
└── README_DX12.md                  # このファイル
```

## 次のステップ

### GPU PhysXアクセラレーション統合

PhysX GPUビルドを使用している場合、GPU物理演算を有効化できます：

```cpp
// initPhysics()内に追加
PxCudaContextManagerDesc cudaContextManagerDesc;
PxCudaContextManager* cudaContextManager = PxCreateCudaContextManager(
    *gFoundation, cudaContextManagerDesc);

if (cudaContextManager && cudaContextManager->contextIsValid())
{
    sceneDesc.cudaContextManager = cudaContextManager;
    sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
    sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
}
```

### より複雑なシミュレーション

- **ジョイント**: ラグドール、車両、ロボットアームなど
- **ソフトボディ**: 布、ロープ、変形可能なオブジェクト
- **流体**: 水、煙、粒子シミュレーション
- **キャラクターコントローラー**: ゲームキャラクターの移動

### レイトレーシング

DirectX 12のDXRを使用して、リアルタイムレイトレーシングを追加：

- リフレクション（反射）
- シャドウ（影）
- アンビエントオクルージョン
- グローバルイルミネーション

## 参考資料

- [PhysX 5.x Documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.1.0/docs/Index.html)
- [DirectX 12 Programming Guide](https://learn.microsoft.com/en-us/windows/win32/direct3d12/directx-12-programming-guide)
- [HLSL Language Reference](https://learn.microsoft.com/en-us/windows/win32/direct3dhlsl/dx-graphics-hlsl)

## ライセンス

このサンプルコードはPhysX SDKのライセンスに従います。
DirectX 12関連コードはMicrosoftのサンプルを参考にしています。

---

**作成日**: 2026年1月14日
**PhysX バージョン**: 5.6.1
**DirectX バージョン**: 12
