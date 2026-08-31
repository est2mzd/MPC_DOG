#!/usr/bin/env bash
# quad_mujoco.pyのrecording:=true(camera_track_robot:=false)で録画したmp4を、
# 目視確認用のGIFへ変換する。
#
# 背景: 追従カメラの録画は、実際に前進していても画面上は常に「その場」に
# 見えてしまい、CSVの数値だけでは前進を目視確認できないという問題があった
# (2026-08-30、agent_reports/step01/quad_sdk_step01_investigation.md参照)。固定カメラでの
# 録画と組み合わせて、歩行の成否をGIFで目視確認できるようにするための変換。
#
# 使い方: bash scripts/trial/make_gif.sh <入力mp4> <出力gif> [fps] [幅px]
set -euo pipefail

IN_MP4="${1:?使い方: make_gif.sh <入力mp4> <出力gif> [fps] [幅px]}"
OUT_GIF="${2:?使い方: make_gif.sh <入力mp4> <出力gif> [fps] [幅px]}"
FPS="${3:-10}"
WIDTH="${4:-480}"

if [ ! -f "${IN_MP4}" ]; then
  echo "ERROR: 入力mp4が見つかりません: ${IN_MP4}" >&2
  exit 1
fi

PALETTE="$(mktemp --suffix=.png)"
trap 'rm -f "${PALETTE}"' EXIT

# 動画の全体時間[s]を取得し、各フレームに「現在時刻[s] / 最終時刻[s]」を
# 焼き込む。GIFを開いたビューアがアニメーションを再生しない(先頭フレームだけの
# サムネイル表示になる)場合でも、時刻表示自体が静止画として見えることで
# 「動いていない」ように見える原因の切り分けに使える。
TOTAL_S="$(ffprobe -v error -show_entries format=duration -of default=noprint_wrappers=1:nokey=1 "${IN_MP4}")"
TOTAL_S_FMT="$(printf '%.1f' "${TOTAL_S}")"
FONT="/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"
OUT_BASENAME="$(basename "${OUT_GIF}")"
# 不透明な黒背景+太字白文字、大きめのフォントサイズにする
# (白文字+黒背景の方が視認性が高いとのユーザーからのフィードバックを反映)。
# 時刻は小数点1桁に丸める。%{pts\:flt}は桁数を制御できない(6桁固定)ため、
# 整数部(trunc)と小数第1位(mod 1して10倍しtrunc)を%{eif:...:d}で個別に
# 整数フォーマットし、"."で連結する(ユーザーからの「小数点1桁に丸めて」を反映)。
# 左上=時刻、左下=このGIF自身のファイル名(「どのファイルを見ているか分からない」
# というユーザーからのフィードバックを受け、ファイル名自体も画面に焼き込む)。
DRAWTEXT_TIME="drawtext=fontfile=${FONT}:text='t=%{eif\\:trunc(t)\\:d}.%{eif\\:trunc(mod(t\\,1)*10)\\:d} / ${TOTAL_S_FMT}s':x=10:y=10:fontsize=48:fontcolor=white:box=1:boxcolor=black:boxborderw=8"
DRAWTEXT_NAME="drawtext=fontfile=${FONT}:text='${OUT_BASENAME}':x=10:y=h-th-10:fontsize=32:fontcolor=white:box=1:boxcolor=black:boxborderw=8"
DRAWTEXT="${DRAWTEXT_TIME},${DRAWTEXT_NAME}"

# 2パス方式(パレット生成→適用)で、単純な減色より高画質かつ低ファイルサイズにする。
# 時刻・ファイル名の焼き込み(drawtext)はパレット生成・適用の両方で同じ
# フィルタチェーンにする(焼き込んだ文字の背景も含めてパレットに反映させるため)。
ffmpeg -y -i "${IN_MP4}" -vf "${DRAWTEXT},fps=${FPS},scale=${WIDTH}:-1:flags=lanczos,palettegen" "${PALETTE}" -loglevel error
ffmpeg -y -i "${IN_MP4}" -i "${PALETTE}" \
  -filter_complex "${DRAWTEXT},fps=${FPS},scale=${WIDTH}:-1:flags=lanczos[x];[x][1:v]paletteuse" \
  -loop 0 "${OUT_GIF}" -loglevel error

echo "Wrote GIF: ${OUT_GIF} ($(du -h "${OUT_GIF}" | cut -f1))"
