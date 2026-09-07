#!/bin/sh

set -eu

version=431d58d5613fd8fae38afe50282b25302de84bf7
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
build_dir=${project_dir}/build
source_dir=${1:-${build_dir}/btstack-source}
output=${2:-${build_dir}/btstack-s31-notices.tar.xz}
package_dir=${project_dir}/buildroot-external/package/btstack-s31

resolved_output=$(realpath -m "$output")
case "$resolved_output" in
	"${build_dir}"/*) ;;
	*)
		echo "Refusing BTstack notice output outside ${build_dir}: ${resolved_output}" >&2
		exit 1
		;;
esac

[ -r "${source_dir}/.s31-btstack-version" ] || {
	echo "Missing pinned BTstack source marker in ${source_dir}" >&2
	exit 1
}
[ "$(cat "${source_dir}/.s31-btstack-version")" = "$version" ] || {
	echo "BTstack source does not match pinned commit ${version}" >&2
	exit 1
}
[ -s "${source_dir}/LICENSE" ] || {
	echo "Missing upstream BTstack LICENSE" >&2
	exit 1
}

mkdir -p "$(dirname -- "$resolved_output")"
temporary=$(mktemp -d "${build_dir}/btstack-notices.XXXXXX")
staging=${temporary}/btstack-s31-notices
trap 'rm -rf "$temporary"' EXIT HUP INT TERM

mkdir -p "${staging}/downstream"
cp "${source_dir}/LICENSE" "${staging}/BTSTACK_LICENSE"
cp "${package_dir}/0001-linux-allow-no-packet-log-and-persist-tlv.patch" \
	"${package_dir}/0002-a2dp-add-low-overhead-transport-mode.patch" \
	"${package_dir}/Config.in" \
	"${package_dir}/btstack-s31.mk" \
	"${package_dir}/s31_btstack_config.h" \
	"${staging}/downstream/"
printf '%s\n' "$version" >"${staging}/BTSTACK_COMMIT"
printf '%s\n' \
	'BTstack source: https://github.com/bluekitchen/btstack' \
	"Pinned commit: ${version}" \
	'' \
	'This bundle contains the upstream license and the downstream build and patch' \
	'inputs used by the ESP32-S31 image. BTstack permits personal/non-commercial' \
	'use under the included license; commercial use requires a separate license.' \
	>"${staging}/SOURCE.txt"

tar -C "$temporary" -cJf "$resolved_output" btstack-s31-notices
printf 'BTstack notice bundle: %s\n' "$resolved_output"
