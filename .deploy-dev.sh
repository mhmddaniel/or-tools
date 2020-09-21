#!/bin/bash
now="$(date +'%Y%m%d%M%S')"

read_var() {
     VAR=$(grep -w $1 $2 | xargs)
     IFS="=" read -ra VAR <<< "$VAR"
     echo ${VAR[1]}
}
 
VERSION=$(read_var VERSION .version)
echo ${VERSION}

docker build -t registry.gitlab.com/paketid/route-optimization:dev-${VERSION}-${now} -f Dockerfile .
docker push registry.gitlab.com/paketid/route-optimization:dev-${VERSION}-${now}
kubectl --kubeconfig="/home/gitlab-runner/.kube/k8s-cluster-config" config use-context dev
kubectl --kubeconfig="/home/gitlab-runner/.kube/k8s-cluster-config" set image deployment/route-optimization-dev  route-optimization-dev=registry.gitlab.com/paketid/route-optimization:dev-${VERSION}-${now}
