. /etc/os-release && \
apt-get update && \
apt-get install -y ca-certificates curl gnupg2 && \
echo "deb http://download.opensuse.org/repositories/devel:/kubic:/libcontainers:/stable/xUbuntu_${VERSION_ID}/ /" > /etc/apt/sources.list.d/devel:kubic:libcontainers:stable.list && \
curl -fsL "https://download.opensuse.org/repositories/devel:kubic:libcontainers:stable/xUbuntu_${VERSION_ID}/Release.key" | apt-key add - && \
apt-get update && \
apt-get install -y buildah podman