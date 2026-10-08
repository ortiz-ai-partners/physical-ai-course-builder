# 夜間開発の引き継ぎ

ユーザー承認：2026-10-09朝まで判断が不要な開発を続ける。GitHubとNotionへの更新も許可済み。課金・実機操作はしない。APIを新規契約せず、このチャットのAIを上位計画役として使う方針。

## 現在の動作

- 手動フォーク運搬、履帯外観、記録処理の同期修正。
- 2箱の日本語配置エディター。ユーザーが保存・3D表示を確認済み。
- 赤箱1個の直線運搬。目標のY座標に合わせた初期配置で開始し、状態フィードバックで運ぶ。施工中の座標変更や固定拘束はない。
- AIがこのチャットで命名・設計した「ふたつの灯台ゲート」の計画JSONを検査し、完成状態のコースを自動走行。これは**施工とまだ接続されていない**。
- AI計画の実行結果をJSONで返す。保存済み計画の再生は新しいAI推論とは表示しない。
- facility_spec.jsonに、ユーザー提供の大きなボール1500個・海のぬいぐるみ14匹の情報を保存。直径・施設寸法・種類は未確認。

## 検証済み

- verify.py：前後進・左右旋回・昇降・フォーク接触運搬・解放・記録同期。
- verify_layout.py：無効配置、保存・再読込、運搬起動リクエスト、二重起動抑止。ネットワーク権限のない環境ではHTTP試験は失敗するので結果を区別。
- verify_transport.py：4目標で成功。約3mを17.72秒、設置誤差約1.4cm。脱落を失敗にする。
- verify_navigation.py：4経由点を44.34秒で通過し終点誤差約14.1cm。狭い門は箱への接触で失敗。
- app.py --plan examples/ortiz-gate-plan.json --screenshot ... で最終状態を描画確認。

## 次に進める工程

1. transport.pyを単一箱・直線専用から拡張。旋回／共通資材置き場からの搬入／箱2個の施工を目指す。車両や箱を途中で瞬間移動させない。
2. 同じ物理シーンで施工完了後にNavigatorへ切り替えて通過する。今の完成配置プレビューとの違いを明記。
3. 静的な坂道の接触を実装し、車両が本当に乗れる条件を測定。
4. トランポリンのばね・減衰近似、ボールプールの段階的個数ベンチマーク。外観だけと動的個体を区別。
5. 施設画像はユーザー指定の名称で検索して参考にできるが、人物写真の転用はしない。不明寸法は仮値。

## 実装上の注意

- 低い旋回入力ではスキッドステア車両が向きを変えきれず時間切れになる。navigation.pyは旋回時の最小入力と角度の不感帯を持つ。初回失敗はこれで改善した。
- 元の手動操作フォルダbulldozer-labと、公開用physical-ai-course-builderは別。ユーザーのrecordingsやdesignsを上書き・公開しない。
- エディターはHTML直開きでは保存できない。design.cmdから起動し、黒いコンソールを残す。このPC用のoutputs/design-course.cmdもある。
- UIサーバーが古いままなら再起動が必要。就寝中に大量の可視ウィンドウを起動しない。
- GitHub CLI認証は使えず、GitHub connectorのcreate_tree→create_commit→update_refで更新できた。期待する現在SHAで保護し、既存ファイルを残す。反映後はfetchで比較する。
- 今回の夜間継続は同じチャットのheartbeat automation（id: physical-ai）。2026-10-09朝7時以降は大きな新規実装を始めず、結果をまとめて停止する。

## 起動

公開コードはsetup.cmd後にstart.cmd、design.cmd、drive-plan.cmd。
このPCの作業用Pythonはプロジェクトの2階層上のwork/github-package-check/Scripts/python.exe。追加課金不要。テストは描画なしを優先し、スクリーンショットが必要なときだけ描画。

Notionプロジェクト： https://app.notion.com/p/3f35e0de2f3281dbabfec38aecafe183
GitHub： https://github.com/ortiz-ai-partners/physical-ai-course-builder
