#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

codename="$(grep '^VERSION_CODENAME=' /etc/os-release | sed 's/^VERSION_CODENAME=//')"
case "${codename}" in
    bionic) LLVM_VERSION=13 ;;
    *) LLVM_VERSION=15 ;;
esac
echo "deb http://apt.llvm.org/${codename}/ llvm-toolchain-${codename}-${LLVM_VERSION} main" |
    sudo tee "/etc/apt/sources.list.d/llvm-toolchain-${codename}-${LLVM_VERSION}.list" >/dev/null
curl https://apt.llvm.org/llvm-snapshot.gpg.key |
    gpg --dearmor |
    sudo tee /etc/apt/trusted.gpg.d/llvm-snapshot.gpg >/dev/null

sudo apt-get -qq update
sudo apt-get -o Dpkg::Use-Pty=0 install -y --no-install-recommends \
    clang-"${LLVM_VERSION}" \
    lld-"${LLVM_VERSION}"

for tool in /usr/bin/clang*-"${LLVM_VERSION}" /usr/bin/*lld*-"${LLVM_VERSION}"; do
    link="${tool%"-${LLVM_VERSION}"}"
    sudo update-alternatives --install "${link}" "${link##*/}" "${tool}" 10
done

echo "RUSTFLAGS=${RUSTFLAGS} -C linker=clang-${LLVM_VERSION} -C link-arg=-fuse-ld=lld-${LLVM_VERSION}" >>"${GITHUB_ENV}"
