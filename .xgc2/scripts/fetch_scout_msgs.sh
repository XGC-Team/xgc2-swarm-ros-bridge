#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# Download pinned scout_msgs onto the host. The build container only dpkg -i.
set -euo pipefail

ros_distro="${ROS_DISTRO:?set ROS_DISTRO}"
arch="${XGC2_DEB_ARCH:-$(dpkg --print-architecture)}"
out_dir="${1:?output directory}"
version="0.3.3-10"
base_url="${XGC2_SCOUT_MSGS_APT_BASE_URL:-https://xgc2.apt.xiaokang.ink}"

case "${ros_distro}:${arch}" in
  melodic:amd64)
    distribution="bionic"
    expected_sha256="8577a994e8615c14a8617ddc6ac283dc6e482a19d29e681b7ae8478e6f7dc794"
    ;;
  melodic:arm64)
    distribution="bionic"
    expected_sha256="38bca64bdf2eefa559e80a8e3275965e3123529b5bc7931703b72c243ba34d62"
    ;;
  noetic:amd64)
    distribution="focal"
    expected_sha256="3f4c77e92198a506c9436e02576fb59a6b96b12feeec7fdd6393f51093ffa9a4"
    ;;
  noetic:arm64)
    distribution="focal"
    expected_sha256="d34c5f71389df113bfe8b34bb192bb9d3b986c33791c396d28139b08912b4fb0"
    ;;
  *)
    echo "unsupported scout_msgs target: ${ros_distro}:${arch}" >&2
    exit 1
    ;;
esac

package="ros-${ros_distro}-scout-msgs"
filename="${package}_${version}_${arch}.deb"
url="${base_url%/}/pool/${distribution}/main/r/${package}/${filename}"
mkdir -p "${out_dir}"
dest="${out_dir}/${filename}"
if [[ ! -f "${dest}" ]]; then
  echo "fetch ${url}"
  curl --fail --location --silent --show-error --proto '=https' --tlsv1.2 \
    "${url}" -o "${dest}.partial"
  mv "${dest}.partial" "${dest}"
fi
printf '%s  %s\n' "${expected_sha256}" "${dest}" | sha256sum --check --strict
