# BVH2MMD

BVHモーションとPMXモデルから、モデルの体格に合わせた30fpsのVMDを作成するWindows向けアプリです。ボーン対応・座標系・倍率の設定と、平らな床での接地補正に対応します。

## ダウンロードと使い方

[Windows版](https://github.com/sakanawalk/bvh-mmd-bridge/releases/latest) · [BOOTH](https://sakanawalk.booth.pm/items/9038003)

1. ZIPをすべて展開し、`BVH2MMD.exe`を起動します。
2. 入力BVHと対象モデルのPMXを選びます。
3. ボーン対応・座標系・出力先を確認し、VMDを書き出します。

元のBVH・PMXは変更しません。詳しい操作は[ガイド](GUIDE.txt)またはZIP内の「はじめに.txt」を参照してください。Windows版の変換にはPython、Blender、MotionBuilder、MMD本体のインストールは不要です。

## 対応環境と制約

- Windows 64bit
- 標準的なMMDの脚構成と平らな静止床
- 出力はFKモーション。VMD内でモデルのIKをオフに設定
- 指・表情・髪・衣装の物理、階段・移動床・逆関節・特殊な脚構造は対象外

短い接地や回転中の足は固定を控えます。任意のモデルへの適合は保証しません。変換結果はMMD等で再生して確認してください。
実素材での検証はWindows 11 / MMD 9.32 x64の1モデル・1テイクが対象です。6213フレームの再生・数値検査と、抽出421コマの目視確認を実施しています。全コマの連続目視、別PC・別モデルでの検証は未実施です。自動テストには人工BVHと合成PMXを使用しています。

## 利用条件

本体には[sakanawalk Software License 1.0](LICENSE)が適用されます。

- 個人・法人とも無料で利用できます。
- 本体・改変版の無断再配布や販売は禁止です。
- 作成した動画・モーションの公開、配布、商用利用は可能です。入力素材の利用条件は別途適用されます。

第三者ライブラリにはそれぞれのライセンスが適用されます。実モデル・収録モーション・音楽・MMD本体・Autodesk SDKは同梱していません。過去のMIT版に基づく許諾は遡って取り消しません。確認範囲は[権利・来歴の記録](RIGHTS_REVIEW.txt)に記載しています。第三者による公式製品・推奨製品ではありません。
BOOTHの「コーヒー代」は任意の200円です。無料版とファイル・機能・利用条件は同一です。

## ソースから起動

Python 3.12以上（Tkを含む）で実行します。

```console
python -m pip install -r requirements.txt
python app.py
python -m unittest discover -s . -p "test_*.py" -v
```

不具合は[Issues](https://github.com/sakanawalk/bvh-mmd-bridge/issues)へ報告できます。モデル・モーションを添付する際は配布元の利用条件を確認してください。

## 更新履歴

### 0.3.0-rc1 r6

- 変換中のBVH・PMXの変更を検出し、不完全な結果の確定を防止
- 同梱文書に残っていた本体の旧MIT表記を訂正
