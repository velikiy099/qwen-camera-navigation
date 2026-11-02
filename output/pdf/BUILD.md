# レポートPDFの生成

Node.js 22以上と、日本語の明朝体フォントが必要です。Linuxでは Noto Serif CJK JP、macOSではヒラギノ明朝、Windowsでは游明朝などを使用します。フォントの優先順は `browser.mjs` に定義しています。

このディレクトリで依存パッケージとChromiumを用意します。

```sh
npm ci
npx playwright install chromium
npm run build
```

`説明資料_PAI最終課題*.md` が1つであれば、自動的に入力として選び、同名のPDFを生成します。明示する場合は次のように指定します。

```sh
node build_report.mjs --source 説明資料_PAI最終課題_48052.md
```

本文・見出し・表・キャプションはMarkdownから直接読み込み、Mermaidの図は定義から描画します。参照画像は元ファイルのまま埋め込みます。レイアウトは `report.css` で設定し、ヘッダーは空、フッターはページ番号のみです。本文を生成コードへ転記する必要はありません。

数式は `$...$` を文中に、単独行の `$$` で囲んだLaTeX式を独立行に使えます。KaTeXで描画し、数式用フォントもPDF・プレビュー内に埋め込みます。

`python3 build_report.py --account 48052` も同じ生成処理を呼び出します。任意のChromium実行ファイルを使う場合は `REPORT_BROWSER_EXECUTABLE` を指定できます。

## ワールド図の更新

SDFを変更した場合のみ、Pythonの `reportlab` パッケージと上記のNode.js環境で画像を再生成します。Pythonパッケージは、このディレクトリ内の仮想環境へインストールします（以下はmacOS/Linuxの例）。通常のMarkdownからPDFへの変換には不要です。

```sh
python3 -m venv .venv
.venv/bin/python -m pip install reportlab
.venv/bin/python build_world_topview.py
npm run build
```

`assets/world_topview.svg` とPNG、SDFの出典情報を更新します。図の文字も明朝体です。図は説明用で、モデルの入力には使用しません。
