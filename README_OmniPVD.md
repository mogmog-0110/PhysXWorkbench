# PhysX Sample with OmniPVD

このプロジェクトは、PhysX 5.6.1 SDKとOmniPVDを使用したシミュレーション記録・可視化のサンプルです。

## OmniPVDとは

OmniPVD (Omniverse PhysX Visual Debugger)は、PhysX 5.x以降で推奨されている物理シミュレーションの可視化ツールです。シミュレーションデータを`.ovd`ファイルに記録し、NVIDIA Omniverseで再生・解析できます。

## ビルド方法

```bash
cd PhysXSample/build
cmake --build . --config Debug --target PhysXSampleOmniPVD
```

## 実行方法

### 1. シミュレーションを実行してOVDファイルを生成

```bash
cd build/Debug
PhysXSampleOmniPVD.exe
```

または、出力ファイル名を指定:

```bash
PhysXSampleOmniPVD.exe my_simulation.ovd
```

### 2. 実行結果

プログラムを実行すると、以下のような出力が表示されます:

```
=== PhysX Sample with OmniPVD ===
OmniPVD will write to: physx_simulation.ovd
PhysX initialized successfully!
OmniPVD sampling started successfully!
Ground plane created.
Created 6 dynamic objects.

Starting simulation (10 seconds)...
Recording to OmniPVD file: physx_simulation.ovd
Frame 0 / 600 (Time: 0s)
Frame 100 / 600 (Time: 1.66667s)
Frame 200 / 600 (Time: 3.33333s)
...

Simulation completed!
OmniPVD data saved to: physx_simulation.ovd
PhysX cleaned up.
```

## シミュレーション内容

このサンプルでは以下のオブジェクトが作成されます:

1. **地面**: 静的な無限平面（Y=0）
2. **箱3個**: 高さ10m, 15m, 20mに配置
3. **球2個**: 高さ25m, 30mに配置
4. **大きな箱1個**: 高さ35mに配置

すべてのオブジェクトが重力によって落下し、地面に衝突します。シミュレーションは10秒間（600フレーム）実行されます。

## OmniPVDファイルの可視化

### 方法1: NVIDIA Omniverse（推奨）

1. **NVIDIA Omniverseのインストール**
   - [NVIDIA Omniverse公式サイト](https://www.nvidia.com/en-us/omniverse/)からOmniverse Launcherをダウンロード
   - Omniverse Launcherから以下をインストール:
     - Omniverse USD Composer (旧Create)
     - またはOmniverse Code

2. **OVDファイルを開く**
   - USD ComposerまたはCodeを起動
   - File > Open で生成された`.ovd`ファイルを選択
   - シミュレーションが3Dビューポートに表示されます

3. **再生とコントロール**
   - タイムラインスライダーでシミュレーションを再生
   - カメラを自由に動かして様々な角度から観察
   - オブジェクトを選択して詳細情報を確認

### 方法2: PhysX OmniPVD Viewer（簡易版）

PhysX SDKに含まれるOmniPVDビューアを使用する場合:

```bash
# ビューアの場所（環境によって異なります）
cd PhysX-5.6.1/omni/...
# ビューアを起動してOVDファイルを読み込む
```

## トラブルシューティング

### エラー: OmniPVDサンプリングを開始できない

```
Warning: Could not start OmniPVD sampling
```

**原因**: `PX_SUPPORT_OMNI_PVD`が無効になっている可能性があります。

**解決方法**: PhysX SDKがOmniPVDサポート付きでビルドされているか確認してください。

### ファイルが生成されない

実行ディレクトリに書き込み権限があるか確認してください。相対パスではなく絶対パスを指定してみてください:

```bash
PhysXSampleOmniPVD.exe C:\path\to\output.ovd
```

## プログラムのカスタマイズ

### オブジェクトの追加

`main_omnipvd.cpp`の`main()`関数内でオブジェクトを追加できます:

```cpp
// カプセルを追加
PxRigidDynamic* capsule = PxCreateDynamic(
    *gPhysics,
    PxTransform(PxVec3(3, 40, 3)),
    PxCapsuleGeometry(0.5f, 1.0f),
    *gMaterial,
    1.0f
);
gScene->addActor(*capsule);
objects.push_back(capsule);
```

### シミュレーション時間の変更

```cpp
const int numSteps = 1200; // 20秒間のシミュレーション
```

### 重力の変更

```cpp
sceneDesc.gravity = PxVec3(0.0f, -19.62f, 0.0f); // 2倍の重力
```

## OmniPVDの利点

1. **完全な記録**: シミュレーションのすべてのフレームを記録
2. **後から解析**: 実行後に何度でも再生・解析可能
3. **高品質なビジュアル**: Omniverseの高品質レンダリング
4. **デバッグ機能**: オブジェクトの詳細情報、衝突点、力ベクトルなどを表示
5. **共有が容易**: OVDファイルを共有してチームで解析可能

## 次のステップ

### 1. より複雑なシミュレーション

- ジョイント（関節）の実装
- 複雑なメッシュ形状の使用
- トリガーと衝突フィルタリング

### 2. GPU アクセラレーション

PhysX 5.xはGPUアクセラレーションをサポートしています:

```cpp
// GPUディスパッチャーを追加
PxCudaContextManagerDesc cudaContextManagerDesc;
PxCudaContextManager* cudaContextManager = PxCreateCudaContextManager(
    *gFoundation,
    cudaContextManagerDesc
);
sceneDesc.cudaContextManager = cudaContextManager;
sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
```

### 3. リアルタイムシミュレーション

- インタラクティブな入力処理
- リアルタイム制御
- ゲームループとの統合

## 参考資料

- [PhysX 5.x Documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.1.0/docs/Index.html)
- [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)
- [PhysX GitHub Repository](https://github.com/NVIDIA-Omniverse/PhysX)
- [OmniPVD User Guide](https://docs.omniverse.nvidia.com/)

## ファイル構成

```
PhysXSample/
├── src/
│   ├── main.cpp              # 基本サンプル（PVD接続）
│   └── main_omnipvd.cpp      # OmniPVDサンプル
├── build/
│   └── Debug/
│       ├── PhysXSample.exe           # 基本版
│       ├── PhysXSampleOmniPVD.exe    # OmniPVD版
│       └── physx_simulation.ovd      # 生成されたOVDファイル
├── CMakeLists.txt
├── README.md
└── README_OmniPVD.md         # このファイル
```
