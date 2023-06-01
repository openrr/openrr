#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

codename="$(grep '^VERSION_CODENAME=' /etc/os-release | sed 's/^VERSION_CODENAME=//')"
llvm_version=16
echo "deb http://apt.llvm.org/${codename}/ llvm-toolchain-${codename}-${llvm_version} main" |
    sudo tee "/etc/apt/sources.list.d/llvm-toolchain-${codename}-${llvm_version}.list" >/dev/null
curl https://apt.llvm.org/llvm-snapshot.gpg.key |
    gpg --dearmor |
    sudo tee /etc/apt/trusted.gpg.d/llvm-snapshot.gpg >/dev/null

sudo apt-get -qq update
sudo apt-get -o Dpkg::Use-Pty=0 install -y --no-install-recommends \
    clang-"${llvm_version}" \
    lld-"${llvm_version}"

for tool in /usr/bin/clang*-"${llvm_version}" /usr/bin/*lld*-"${llvm_version}"; do
    link="${tool%"-${llvm_version}"}"
    sudo update-alternatives --install "${link}" "${link##*/}" "${tool}" 10
done

echo "RUSTFLAGS=${RUSTFLAGS} -C linker=clang-${llvm_version} -C link-arg=-fuse-ld=lld-${llvm_version}" >>"${GITHUB_ENV}"
